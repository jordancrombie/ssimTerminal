# QR Code Display Issue - Troubleshooting Guide

## Observed Behavior

1. Terminal boots and shows "Ready for Payment" screen
2. Terminal appears online in `/terminal` interface
3. Payment is initiated from website
4. Server shows "QR code displayed on terminal" with countdown
5. **Terminal screen does NOT change** - stays on "Ready for Payment"
6. Terminal logs show it received the `payment_request` and sent `displayed` status

## Key Clue

After terminal reboot:
- Screen shows "Ready for Payment"
- Server briefly shows terminal as **offline**, then **online**
- This offline→online flicker suggests a connection state issue

## Hypothesis

There may be a race condition or state mismatch when the terminal reconnects:
1. Terminal connects via WebSocket
2. Server marks terminal online
3. Terminal disconnects briefly (or server thinks it did)
4. Terminal reconnects
5. **Server's payment session state may not be properly associated with the new connection**

When a payment is initiated, the server may be sending the `payment_request` to a stale/orphaned WebSocket connection or internal state.

## Terminal-Side Evidence

From serial logs, the terminal correctly:
```
WebSocket message (244 bytes): {"type":"payment_request"...
Received message type: payment_request
Payment request: tpay_xxx, Amount: 2200 CAD
UI: QR screen - Amount: 2200 CAD, URL: https://...
DEBUG: LCD_WIDTH=800, LCD_HEIGHT=480
DEBUG: switchScreen() done
QR generated: 41x41 modules
DEBUG: Canvas created and positioned
DEBUG: showQrScreen() complete
State transition: 5 -> 7
Sent payment_status: tpay_xxx = displayed
```

The terminal IS receiving the message and processing it correctly. It sends back `displayed` status which the server acknowledges.

## Questions for Server Team

1. **Connection Management**: When a terminal reconnects, is the old WebSocket connection properly cleaned up? Could there be two connections briefly?

2. **Session Association**: Is the terminal's payment session properly associated with the current WebSocket connection?

3. **Message Routing**: When sending `payment_request`, how does the server determine which WebSocket to use?

4. **Timing**: Is there a minimum delay required between terminal coming online and accepting payments?

5. **State Reset**: Does the server reset the terminal's state on reconnection?

## Suggested Server-Side Logging

Add logging for:
- WebSocket connect/disconnect events with terminal ID
- Number of active connections per terminal ID
- Payment request routing (which connection received it)
- State transitions for terminal sessions

## Reproduction Steps

1. Power cycle the terminal
2. Wait for terminal to show "Ready for Payment"
3. Check `/terminal` - note if it flickers offline→online
4. **Wait a few seconds** after it shows online
5. Initiate payment
6. Observe if QR code displays

## Potential Workaround (to test)

On the terminal side, we could add a delay before marking the terminal as "ready" after WebSocket connection is established. But this is a band-aid - the server should handle reconnection gracefully.

## Terminal Firmware Version

- Version: 0.5.0
- Board: ESP32-S3-Touch-LCD-7 (800x480)
- WebSocket endpoint: `wss://{server}/terminal/ws?apiKey={key}`

---

## Server-Side Analysis (2025-12-21)

### Race Condition Found

There's a bug in the SSIM server's WebSocket connection management that can cause intermittent message delivery failures.

**Root Cause:** The `unregisterTerminalConnection` function doesn't verify which WebSocket is being unregistered.

**Scenario:**
1. Terminal has active WebSocket **WS1**
2. Terminal reconnects (network hiccup, WiFi reconnect, etc.)
3. New WebSocket **WS2** connects successfully
4. `registerTerminalConnection(terminalId, WS2)` **overwrites** WS1 in the Map
5. WS2 is now correctly registered ✓
6. **But then** WS1's `close` event fires (delayed due to network/async timing)
7. `unregisterTerminalConnection(terminalId)` is called
8. This **removes WS2's entry** from the Map! ✗
9. Map is now empty for this terminal
10. Payment requests fail: `connectedTerminals.get(terminalId)` returns `undefined`

**Why this explains the symptoms:**
- **Intermittent**: Depends on timing of old connection's close event
- **Offline→online flicker**: Indicates a reconnection where this race can occur
- **Sometimes works, sometimes doesn't**: If close event fires before reconnect completes, it's fine. If close event is delayed, the new connection is wiped.

### Current Code (Bug)

```typescript
// terminal.ts
export function unregisterTerminalConnection(terminalId: string): void {
  connectedTerminals.delete(terminalId);  // Deletes regardless of which WS!
}
```

### Proposed Fix

```typescript
// terminal.ts
export function unregisterTerminalConnection(terminalId: string, ws: unknown): void {
  const current = connectedTerminals.get(terminalId);
  // Only delete if the WebSocket being closed is the one currently registered
  if (current && current.ws === ws) {
    connectedTerminals.delete(terminalId);
    console.log(`[Terminal] WebSocket connection unregistered for terminal ${terminalId}`);
  } else {
    console.log(`[Terminal] Ignoring unregister for stale connection for terminal ${terminalId}`);
  }
}

// terminal-websocket.ts - update the close handler
ws.on('close', async () => {
  console.log(`[Terminal WS] Terminal disconnected: ${terminal.name} (${terminal.id})`);
  terminalService.unregisterTerminalConnection(terminal.id, ws);  // Pass the ws reference
  await terminalService.updateTerminalStatus(terminal.id, 'offline');
});
```

### Answers to Terminal Team Questions

1. **Connection Management**: Yes, there IS a bug. Old connection's close event can remove the new connection's entry.

2. **Session Association**: Payment sessions are stored separately (in-memory Map by paymentId). The issue is that the WebSocket connection entry gets incorrectly removed.

3. **Message Routing**: Server looks up `connectedTerminals.get(terminalId)` and sends to that WebSocket. If the entry was deleted by the race condition, this returns undefined and the message is never sent.

4. **Timing**: Yes, there's a window of vulnerability. If you wait long enough for the old connection's close event to fully process before the reconnection, it should be safer. This is why "wait a few seconds" sometimes helps.

5. **State Reset**: The server doesn't explicitly reset state on reconnection - it just overwrites the Map entry. The problem is the stale close event undoing this.

### Recommended Testing

After applying the fix:
1. Rapidly connect/disconnect the terminal several times
2. Pull the network cable/disable WiFi briefly, then restore
3. Reboot terminal while server is under load
4. Verify messages are delivered consistently

### Additional Logging Added

The server now logs connection registration/unregistration with terminal ID and total connection count. Look for:
```
[Terminal] WebSocket connection registered for terminal XXX, total connections: N
[Terminal] WebSocket connection unregistered for terminal XXX, remaining connections: N
```

If you see a register immediately followed by an unregister (same terminal ID), that confirms this race condition.

---

## Terminal-Side Improvements (Proposed)

While the server fix addresses the root cause, the terminal can be made more robust to handle edge cases and provide better diagnostics.

### Current Terminal WebSocket Behavior

```cpp
// On connect: immediately transition to IDLE and send heartbeat
case WStype_CONNECTED:
    Serial.printf("WebSocket connected to: %s\n", (char *)payload);
    transitionTo(TerminalState::IDLE);
    sendHeartbeat();
    break;

// Reconnect settings
webSocket.setReconnectInterval(5000);  // 5 second reconnect delay
webSocket.enableHeartbeat(15000, 3000, 2);  // ping every 15s, 3s timeout, 2 retries
```

### Proposed Improvements

#### 1. Wait for Server Acknowledgment Before "Ready"

Instead of immediately transitioning to IDLE on connect, wait for the `config_update` message from the server which confirms the connection is fully established.

```cpp
case WStype_CONNECTED:
    Serial.printf("WebSocket connected to: %s\n", (char *)payload);
    transitionTo(TerminalState::CONNECTING);  // Stay in connecting state
    sendHeartbeat();
    break;

// In handleServerMessage():
if (strcmp(type, "config_update") == 0) {
    Serial.println("Config update received - connection confirmed");
    if (currentState == TerminalState::CONNECTING) {
        transitionTo(TerminalState::IDLE);  // NOW we're ready
    }
}
```

**Benefit:** Ensures server has fully registered the connection before accepting payments.

#### 2. Add Connection Session ID

Generate a unique session ID on each connection and include it in heartbeats. Helps with debugging which connection is which.

```cpp
static String connectionSessionId;

void connectWebSocket() {
    // Generate unique session ID
    connectionSessionId = String(millis()) + "_" + String(random(1000, 9999));
    Serial.printf("New connection session: %s\n", connectionSessionId.c_str());
    // ... rest of connect code
}

void sendHeartbeat() {
    // ... existing code ...
    payload["sessionId"] = connectionSessionId;
}
```

**Benefit:** Server can log session IDs to identify stale vs current connections.

#### 3. Graceful Disconnect Before Reconnect

If we detect a connection issue, explicitly close the connection before allowing reconnect.

```cpp
void forceReconnect() {
    Serial.println("Forcing reconnect...");
    webSocket.disconnect();
    delay(1000);  // Give server time to process disconnect
    connectWebSocket();
}
```

**Benefit:** Gives server a clean disconnect event before the reconnect.

#### 4. Heartbeat Response Tracking

Track if we're getting `pong` responses. If we send multiple heartbeats without pong, force reconnect.

```cpp
static int heartbeatsSentWithoutPong = 0;
static const int MAX_MISSED_PONGS = 3;

void sendHeartbeat() {
    heartbeatsSentWithoutPong++;
    if (heartbeatsSentWithoutPong > MAX_MISSED_PONGS) {
        Serial.println("WARNING: No pong received for multiple heartbeats, forcing reconnect");
        forceReconnect();
        return;
    }
    // ... send heartbeat
}

// In handleServerMessage():
if (strcmp(type, "pong") == 0) {
    heartbeatsSentWithoutPong = 0;  // Reset counter
}
```

**Benefit:** Detects zombie connections that appear connected but aren't receiving messages.

#### 5. Post-Connect Stabilization Delay

Add a small delay after connection before sending any messages, to allow server to fully set up.

```cpp
case WStype_CONNECTED:
    Serial.printf("WebSocket connected, waiting for stabilization...\n");
    delay(500);  // 500ms stabilization delay
    transitionTo(TerminalState::CONNECTING);
    sendHeartbeat();
    break;
```

**Benefit:** Reduces chance of messages being lost during server-side connection setup.

### Implementation Priority

1. **High**: Wait for `config_update` before IDLE (improvement #1)
2. **Medium**: Heartbeat response tracking (improvement #4)
3. **Low**: Connection session ID (improvement #2) - mainly for debugging
4. **Low**: Graceful disconnect (improvement #3) - belt and suspenders
5. **Optional**: Post-connect delay (improvement #5) - only if #1 isn't sufficient

### Performance Considerations

- Improvement #1 adds minimal latency (server sends `config_update` immediately on connect)
- Improvement #4 adds no latency, just tracking
- Improvement #5 adds 500ms latency on connect (acceptable for robustness)
- None of these affect steady-state performance during normal operation

---

## Server Team Feedback on Terminal Proposals (2025-12-21)

### Overall Assessment

These are solid defensive improvements. The server-side race condition fix addresses the root cause, but defense in depth is good practice. Here's my feedback on each:

### Improvement #1: Wait for `config_update` - ✅ STRONGLY RECOMMENDED

This is excellent and I fully support it. The server sends `config_update` immediately after `handleConnection` completes, which means:
1. Terminal is registered in the connection Map
2. Database status is updated to 'online'
3. Welcome message is sent

Waiting for this confirmation ensures the terminal doesn't accept payments until the server is fully ready. **This should be priority #1.**

**Server-side note:** The `config_update` is sent at line 190-196 of `terminal-websocket.ts` immediately after connection setup. It's reliable and fast.

### Improvement #2: Connection Session ID - ✅ RECOMMENDED (for debugging)

This would be very helpful for debugging. I can add server-side support to:
1. Log the session ID when received in heartbeats
2. Store it in the connection registry for correlation

**Proposal:** I'll add support on the server to log and track session IDs if you implement this. Format suggestion:
```json
{
  "type": "heartbeat",
  "payload": {
    "sessionId": "12345_6789",
    "firmwareVersion": "0.5.0",
    ...
  }
}
```

### Improvement #3: Graceful Disconnect Before Reconnect - ⚠️ OPTIONAL

With the server-side race condition fix now in place, this is less critical. The server now correctly ignores stale close events.

However, it's still good practice. The 1-second delay is reasonable but could be reduced to 500ms or even 200ms - the server processes close events quickly.

**Note:** Be careful with `delay()` in the main loop - if you're using FreeRTOS tasks, consider `vTaskDelay()` to avoid blocking other tasks.

### Improvement #4: Heartbeat Response Tracking - ✅ RECOMMENDED

This is excellent for detecting zombie connections. The server ALWAYS responds to heartbeats with `pong` (line 215 of `terminal-websocket.ts`), so missing pongs definitively indicates a dead connection.

**Suggestion:** Consider also tracking time since last pong, not just count. If you send heartbeats every 30s and miss 3 pongs, that's 90+ seconds of dead connection.

```cpp
static unsigned long lastPongReceivedAt = 0;
const unsigned long PONG_TIMEOUT_MS = 120000;  // 2 minutes

// In handleServerMessage():
if (strcmp(type, "pong") == 0) {
    lastPongReceivedAt = millis();
    heartbeatsSentWithoutPong = 0;
}

// In loop or heartbeat check:
if (millis() - lastPongReceivedAt > PONG_TIMEOUT_MS) {
    forceReconnect();
}
```

### Improvement #5: Post-Connect Stabilization Delay - ❌ NOT RECOMMENDED

With improvement #1 (waiting for `config_update`), this becomes unnecessary. The `config_update` message IS the stabilization signal - it means the server is ready.

Adding a fixed delay:
- Adds latency without benefit
- Doesn't actually guarantee anything (could still be too short or too long)
- Is a "magic number" approach vs proper handshaking (#1)

**Skip this if you implement #1.**

### Revised Priority Recommendation

1. **MUST HAVE**: Wait for `config_update` before IDLE (#1)
2. **SHOULD HAVE**: Heartbeat/pong tracking (#4)
3. **NICE TO HAVE**: Connection session ID (#2) - implement when debugging is needed
4. **OPTIONAL**: Graceful disconnect (#3) - belt and suspenders
5. **SKIP**: Post-connect delay (#5) - redundant with #1

### Server-Side Enhancements - IMPLEMENTED ✅

The following server-side support has been added:

1. **Session ID logging** ✅ - Server now logs session ID from heartbeats and tracks changes
   - Logs: `[Terminal] Session ID changed for XXX: none -> 12345_6789`
   - Stored in connection registry for correlation

2. **Stale connection alerts** ✅ - Server logs when stale close events are ignored
   - Logs: `[Terminal] Ignoring unregister for stale connection for XXX (current session: 12345_6789)`

3. **Connection duration tracking** - Can be added if needed

### Testing Checklist

After implementing changes, test these scenarios:
- [ ] Normal boot → connect → payment → success
- [ ] Rapid WiFi toggle (disconnect/reconnect quickly)
- [ ] Payment during reconnection (should queue or reject cleanly)
- [ ] Long idle period (1+ hours) → payment works
- [ ] Server restart while terminal connected → terminal reconnects and works
