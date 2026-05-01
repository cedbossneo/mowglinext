/**
 * Noise IK initiator over WebSocket.
 *
 * Wire format per protocol.go:
 *   [u16 BE length][AEAD ciphertext]
 *
 * Pattern: Noise_IK_25519_ChaChaPoly_BLAKE2s
 *   <- s          (responder static known out-of-band)
 *   -> e, es, s, ss, [payload]   (handshake message 1, initiator → responder)
 *   <- e, ee, se, [payload]      (handshake message 2, responder → initiator)
 *
 * After handshake, transport messages are CBOR-encoded Frame objects wrapped in
 * [u16 BE length][ChaChaPoly ciphertext].
 */

import { generateKeyPair, sharedKey } from '@stablelib/x25519';
import { ChaCha20Poly1305 } from '@stablelib/chacha20poly1305';
import { BLAKE2s } from '@stablelib/blake2s';
import { randomBytes } from '@stablelib/random';
import { encode as cborEncode, decode as cborDecode } from 'cbor-x';

// ── Constants ────────────────────────────────────────────────────────────────

const PROTOCOL_NAME = 'Noise_IK_25519_ChaChaPoly_BLAKE2s';
const DHLEN = 32;
const PROTOCOL_NAME_BYTES = new TextEncoder().encode(PROTOCOL_NAME);

// ── Types ────────────────────────────────────────────────────────────────────

export interface HandshakeReply {
  robot_id: string;
  robot_name: string;
  version: string;
  ts: number;
}

type FrameType = 'req' | 'res' | 'wso' | 'wsa' | 'wse' | 'wsd' | 'wsc' | 'ping' | 'pong';

interface Frame {
  t: FrameType;
  id: number;
  method?: string;
  path?: string;
  headers?: Record<string, string>;
  body?: Uint8Array;
  status?: number;
  data?: Uint8Array;
  binary?: boolean;
  code?: number;
  reason?: string;
  ts?: number;
}

interface PendingRequest {
  resolve: (frame: Frame) => void;
  reject: (err: Error) => void;
  timer: ReturnType<typeof setTimeout>;
}

interface StreamHandlers {
  onData: (data: Uint8Array, binary: boolean) => void;
  onClose: (code: number, reason: string) => void;
  onError: (err: Error) => void;
}

// ── BLAKE2s helpers ──────────────────────────────────────────────────────────

function blake2sMac(data: Uint8Array, key: Uint8Array): Uint8Array {
  const h = new BLAKE2s(32, { key });
  h.update(data);
  return h.digest();
}

function blake2sHash(data: Uint8Array): Uint8Array {
  const h = new BLAKE2s(32);
  h.update(data);
  return h.digest();
}

function hkdf2(
  chainingKey: Uint8Array,
  inputKeyMaterial: Uint8Array,
): [Uint8Array, Uint8Array] {
  const tempKey = blake2sMac(inputKeyMaterial, chainingKey);
  const out1 = blake2sMac(new Uint8Array([0x01]), tempKey);
  const in2 = new Uint8Array(out1.length + 1);
  in2.set(out1);
  in2[out1.length] = 0x02;
  const out2 = blake2sMac(in2, tempKey);
  return [out1, out2];
}

export function hkdf3(
  chainingKey: Uint8Array,
  inputKeyMaterial: Uint8Array,
): [Uint8Array, Uint8Array, Uint8Array] {
  const tempKey = blake2sMac(inputKeyMaterial, chainingKey);
  const out1 = blake2sMac(new Uint8Array([0x01]), tempKey);
  const in2 = new Uint8Array(out1.length + 1);
  in2.set(out1);
  in2[out1.length] = 0x02;
  const out2 = blake2sMac(in2, tempKey);
  const in3 = new Uint8Array(out2.length + 1);
  in3.set(out2);
  in3[out2.length] = 0x03;
  const out3 = blake2sMac(in3, tempKey);
  return [out1, out2, out3];
}

// ── CipherState ──────────────────────────────────────────────────────────────

class CipherState {
  private k: Uint8Array | null = null;
  private n = 0;

  initializeKey(key: Uint8Array): void {
    this.k = key.slice(0, 32);
    this.n = 0;
  }

  hasKey(): boolean {
    return this.k !== null;
  }

  encryptWithAd(ad: Uint8Array, plaintext: Uint8Array): Uint8Array {
    if (!this.k) return plaintext;
    const nonce = this._nonce();
    const cipher = new ChaCha20Poly1305(this.k);
    const ct = cipher.seal(nonce, plaintext, ad);
    this.n++;
    return ct;
  }

  decryptWithAd(ad: Uint8Array, ciphertext: Uint8Array): Uint8Array {
    if (!this.k) return ciphertext;
    const nonce = this._nonce();
    const cipher = new ChaCha20Poly1305(this.k);
    const pt = cipher.open(nonce, ciphertext, ad);
    if (!pt) throw new Error('Noise: AEAD decryption failed (bad MAC or wrong key)');
    this.n++;
    return pt;
  }

  private _nonce(): Uint8Array {
    // ChaCha20Poly1305: 12-byte nonce, counter in bytes 4–11 (little-endian u64)
    const nonce = new Uint8Array(12);
    const view = new DataView(nonce.buffer);
    view.setUint32(4, this.n >>> 0, true);
    view.setUint32(8, Math.floor(this.n / 0x100000000), true);
    return nonce;
  }
}

// ── SymmetricState ───────────────────────────────────────────────────────────

class SymmetricState {
  private cs = new CipherState();
  private ck: Uint8Array;
  private h: Uint8Array;

  constructor(protocolName: Uint8Array) {
    if (protocolName.length <= DHLEN) {
      this.h = new Uint8Array(DHLEN);
      this.h.set(protocolName);
    } else {
      this.h = blake2sHash(protocolName);
    }
    this.ck = new Uint8Array(this.h);
  }

  mixKey(inputKeyMaterial: Uint8Array): void {
    const [ck, tempK] = hkdf2(this.ck, inputKeyMaterial);
    this.ck = ck;
    this.cs.initializeKey(tempK.slice(0, 32));
  }

  mixHash(data: Uint8Array): void {
    const combined = concat(this.h, data);
    this.h = blake2sHash(combined);
  }

  encryptAndHash(plaintext: Uint8Array): Uint8Array {
    const ct = this.cs.encryptWithAd(this.h, plaintext);
    this.mixHash(ct);
    return ct;
  }

  decryptAndHash(ciphertext: Uint8Array): Uint8Array {
    const pt = this.cs.decryptWithAd(this.h, ciphertext);
    this.mixHash(ciphertext);
    return pt;
  }

  split(): [CipherState, CipherState] {
    const [tempK1, tempK2] = hkdf2(this.ck, new Uint8Array(0));
    const c1 = new CipherState();
    const c2 = new CipherState();
    c1.initializeKey(tempK1.slice(0, 32));
    c2.initializeKey(tempK2.slice(0, 32));
    return [c1, c2];
  }

  getH(): Uint8Array {
    return new Uint8Array(this.h);
  }
}

// ── NoiseStream ──────────────────────────────────────────────────────────────

export class NoiseStream {
  private dataHandlers: Array<(payload: { data: Uint8Array; binary: boolean }) => void> = [];
  private closeHandlers: Array<(payload: { code: number; reason: string }) => void> = [];
  private errorHandlers: Array<(err: Error) => void> = [];

  constructor(
    public readonly id: number,
    private readonly sendFn: (frame: Frame) => void,
  ) {}

  send(binary: boolean, data: Uint8Array): void {
    this.sendFn({ t: 'wsd', id: this.id, data, binary });
  }

  close(code = 1000, reason = ''): void {
    this.sendFn({ t: 'wsc', id: this.id, code, reason });
  }

  on(event: 'data', cb: (payload: { data: Uint8Array; binary: boolean }) => void): void;
  on(event: 'closed', cb: (payload: { code: number; reason: string }) => void): void;
  on(event: 'error', cb: (err: Error) => void): void;
  on(event: string, cb: unknown): void {
    if (event === 'data') this.dataHandlers.push(cb as (p: { data: Uint8Array; binary: boolean }) => void);
    else if (event === 'closed') this.closeHandlers.push(cb as (p: { code: number; reason: string }) => void);
    else if (event === 'error') this.errorHandlers.push(cb as (e: Error) => void);
  }

  /** @internal */
  _emitData(data: Uint8Array, binary: boolean): void {
    this.dataHandlers.forEach((h) => h({ data, binary }));
  }

  /** @internal */
  _emitClose(code: number, reason: string): void {
    this.closeHandlers.forEach((h) => h({ code, reason }));
  }

  /** @internal */
  _emitError(err: Error): void {
    this.errorHandlers.forEach((h) => h(err));
  }
}

// ── NoiseSession ─────────────────────────────────────────────────────────────

export interface NoiseSessionOptions {
  wsUrl: string;
  peerStaticPub: Uint8Array;
  idToken: () => Promise<string>;
  appVersion: string;
}

type SessionState = 'connecting' | 'handshake' | 'transport' | 'closed';

export class NoiseSession {
  private ws: WebSocket | null = null;
  private sendCs: CipherState | null = null;
  private recvCs: CipherState | null = null;
  private idCounter = 0;
  private pendingRequests = new Map<number, PendingRequest>();
  private streams = new Map<number, StreamHandlers>();

  private closedHandlers: Array<() => void> = [];
  private errorHandlers: Array<(err: Error) => void> = [];
  private reconnectHandlers: Array<() => void> = [];
  private connectedHandlers: Array<(reply: HandshakeReply) => void> = [];

  private reconnectDelay = 1000;
  private sessionClosed = false;
  private pingTimer: ReturnType<typeof setInterval> | null = null;
  private pongDeadline: ReturnType<typeof setTimeout> | null = null;
  private handshakeReply: HandshakeReply | null = null;
  private state: SessionState = 'connecting';

  // Reassemble partial WebSocket binary messages
  private recvBuf: Uint8Array<ArrayBuffer> = new Uint8Array(new ArrayBuffer(0));

  constructor(private readonly opts: NoiseSessionOptions) {}

  async connect(): Promise<HandshakeReply> {
    return new Promise((resolve, reject) => {
      this._doConnect(resolve, reject);
    });
  }

  private _doConnect(
    resolveHandshake: (r: HandshakeReply) => void,
    rejectHandshake: (e: Error) => void,
  ): void {
    this.state = 'connecting';
    const ws = new WebSocket(this.opts.wsUrl);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;

    let phase: 'hs1' | 'hs2' | 'transport' = 'hs1';
    let ss!: SymmetricState;
    let localEphemeral!: ReturnType<typeof generateKeyPair>;
    // Per-session static identity (not persisted — IK still hides it from the relay)
    let localStatic!: ReturnType<typeof generateKeyPair>;

    ws.onopen = () => {
      this.state = 'handshake';
      void (async () => {
        try {
          localEphemeral = generateKeyPair();
          localStatic = generateKeyPair();
          ss = new SymmetricState(PROTOCOL_NAME_BYTES);

          // mix prologue (empty)
          ss.mixHash(new Uint8Array(0));
          // pre-message: mix responder static public key
          ss.mixHash(this.opts.peerStaticPub);

          // write e
          ss.mixHash(localEphemeral.publicKey);
          let msg: Uint8Array = new Uint8Array(localEphemeral.publicKey);

          // es: DH(e, rs)
          const es = sharedKey(localEphemeral.secretKey, this.opts.peerStaticPub);
          ss.mixKey(es);

          // s: encrypt initiator static public key
          const encS = ss.encryptAndHash(localStatic.publicKey);
          msg = concat(msg, encS);

          // ss: DH(s, rs)
          const ssKey = sharedKey(localStatic.secretKey, this.opts.peerStaticPub);
          ss.mixKey(ssKey);

          // payload: CBOR { fb_id_token, nonce, app_version }
          const token = await this.opts.idToken();
          const nonce = randomBytes(32);
          const payload = cborEncode({
            fb_id_token: token,
            nonce,
            app_version: this.opts.appVersion,
          });
          const encPayload = ss.encryptAndHash(payload);
          msg = concat(msg, encPayload);

          phase = 'hs2';
          this._wsSend(ws, msg);
        } catch (err) {
          rejectHandshake(err instanceof Error ? err : new Error(String(err)));
          ws.close();
        }
      })();
    };

    ws.onmessage = (ev: MessageEvent<ArrayBuffer>) => {
      const data = new Uint8Array(ev.data);

      if (phase === 'hs2') {
        try {
          let offset = 0;

          // read re (responder ephemeral)
          const re = data.slice(offset, offset + DHLEN);
          offset += DHLEN;
          ss.mixHash(re);

          // ee: DH(e_i, e_r)
          const ee = sharedKey(localEphemeral.secretKey, re);
          ss.mixKey(ee);

          // se: DH(s_i, e_r)
          const se = sharedKey(localStatic.secretKey, re);
          ss.mixKey(se);

          // decrypt responder payload
          const encPayload = data.slice(offset);
          const payloadBytes = ss.decryptAndHash(encPayload);
          const reply = cborDecode(payloadBytes) as HandshakeReply;

          // split: initiator uses c1 for send, c2 for recv
          const [c1, c2] = ss.split();
          this.sendCs = c1;
          this.recvCs = c2;

          phase = 'transport';
          this.state = 'transport';
          this.handshakeReply = reply;
          this.reconnectDelay = 1000;
          this._startHeartbeat();
          resolveHandshake(reply);
          this.connectedHandlers.forEach((h) => h(reply));
        } catch (err) {
          const e = err instanceof Error ? err : new Error(String(err));
          rejectHandshake(e);
          ws.close();
        }
        return;
      }

      if (phase === 'transport') {
        this._processTransportData(data);
      }
    };

    ws.onerror = () => {
      const err = new Error('WebSocket connection error');
      if (this.state !== 'transport') {
        rejectHandshake(err);
      }
      this.errorHandlers.forEach((h) => h(err));
    };

    ws.onclose = () => {
      this._stopHeartbeat();
      this.sendCs = null;
      this.recvCs = null;
      if (!this.sessionClosed) {
        this._scheduleReconnect(resolveHandshake, rejectHandshake);
      } else {
        this.state = 'closed';
        this.closedHandlers.forEach((h) => h());
      }
    };
  }

  private _processTransportData(incoming: Uint8Array): void {
    this.recvBuf = concat(this.recvBuf, incoming);

    while (this.recvBuf.length >= 2) {
      const len = ((this.recvBuf[0]! << 8) | this.recvBuf[1]!) >>> 0;
      if (this.recvBuf.length < 2 + len) break;

      const ciphertext = this.recvBuf.slice(2, 2 + len);
      this.recvBuf = this.recvBuf.slice(2 + len);

      try {
        const plaintext = this.recvCs!.decryptWithAd(new Uint8Array(0), ciphertext);
        const frame = cborDecode(plaintext) as Frame;
        this._handleFrame(frame);
      } catch (err) {
        this.errorHandlers.forEach((h) =>
          h(err instanceof Error ? err : new Error(String(err))),
        );
      }
    }
  }

  private _handleFrame(frame: Frame): void {
    switch (frame.t) {
      case 'res': {
        const pending = this.pendingRequests.get(frame.id);
        if (pending) {
          clearTimeout(pending.timer);
          this.pendingRequests.delete(frame.id);
          pending.resolve(frame);
        }
        break;
      }
      case 'wsa': {
        // Stream open acknowledged — streams map already set up in openStream()
        // signal waiting promise via a synthetic data event with empty payload
        const handlers = this.streams.get(frame.id);
        if (handlers) {
          handlers.onData(new Uint8Array(0), false);
        }
        break;
      }
      case 'wse': {
        const handlers = this.streams.get(frame.id);
        if (handlers) {
          handlers.onError(new Error(frame.reason ?? 'Stream open rejected'));
          this.streams.delete(frame.id);
        }
        break;
      }
      case 'wsd': {
        const handlers = this.streams.get(frame.id);
        if (handlers) {
          handlers.onData(frame.data ?? new Uint8Array(0), frame.binary ?? false);
        }
        break;
      }
      case 'wsc': {
        const handlers = this.streams.get(frame.id);
        if (handlers) {
          handlers.onClose(frame.code ?? 1000, frame.reason ?? '');
          this.streams.delete(frame.id);
        }
        break;
      }
      case 'pong': {
        if (this.pongDeadline) {
          clearTimeout(this.pongDeadline);
          this.pongDeadline = null;
        }
        break;
      }
      default:
        break;
    }
  }

  async request(
    method: string,
    path: string,
    headers?: Record<string, string>,
    body?: Uint8Array,
  ): Promise<{ status: number; headers: Record<string, string>; body: Uint8Array }> {
    const id = ++this.idCounter;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pendingRequests.delete(id);
        reject(new Error(`Request timeout: ${method} ${path}`));
      }, 30000);

      this.pendingRequests.set(id, {
        resolve: (resFrame) => {
          resolve({
            status: resFrame.status ?? 0,
            headers: resFrame.headers ?? {},
            body: resFrame.body ?? new Uint8Array(0),
          });
        },
        reject,
        timer,
      });

      try {
        this._sendFrame({ t: 'req', id, method, path, headers, body });
      } catch (err) {
        clearTimeout(timer);
        this.pendingRequests.delete(id);
        reject(err instanceof Error ? err : new Error(String(err)));
      }
    });
  }

  async openStream(
    path: string,
    headers?: Record<string, string>,
  ): Promise<NoiseStream> {
    const id = ++this.idCounter;
    const stream = new NoiseStream(id, (f) => this._sendFrame(f));

    return new Promise((resolve, reject) => {
      let opened = false;

      this.streams.set(id, {
        onData: (_data, _binary) => {
          // First onData (from wsa ack) resolves the promise
          if (!opened) {
            opened = true;
            resolve(stream);
          } else {
            stream._emitData(_data, _binary);
          }
        },
        onClose: (code, reason) => stream._emitClose(code, reason),
        onError: (err) => {
          if (!opened) {
            opened = true;
            reject(err);
          } else {
            stream._emitError(err);
          }
        },
      });

      try {
        this._sendFrame({ t: 'wso', id, path, headers });
      } catch (err) {
        this.streams.delete(id);
        reject(err instanceof Error ? err : new Error(String(err)));
      }
    });
  }

  async ping(): Promise<number> {
    const id = ++this.idCounter;
    const ts = Date.now();
    const start = ts;

    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        reject(new Error('Ping timeout'));
      }, 10000);

      // Override pong handling for this specific ping
      const savedHandle = this._handleFrame.bind(this);
      this._handleFrame = (frame: Frame) => {
        if (frame.t === 'pong') {
          clearTimeout(timer);
          this._handleFrame = savedHandle;
          resolve(Date.now() - start);
        } else {
          savedHandle(frame);
        }
      };

      try {
        this._sendFrame({ t: 'ping', id, ts });
      } catch (err) {
        clearTimeout(timer);
        this._handleFrame = savedHandle;
        reject(err instanceof Error ? err : new Error(String(err)));
      }
    });
  }

  close(): void {
    this.sessionClosed = true;
    this._stopHeartbeat();
    this.ws?.close();
  }

  getState(): SessionState {
    return this.state;
  }

  getHandshakeReply(): HandshakeReply | null {
    return this.handshakeReply;
  }

  on(event: 'closed', cb: () => void): void;
  on(event: 'error', cb: (err: Error) => void): void;
  on(event: 'reconnecting', cb: () => void): void;
  on(event: 'connected', cb: (reply: HandshakeReply) => void): void;
  on(event: string, cb: unknown): void {
    if (event === 'closed') this.closedHandlers.push(cb as () => void);
    else if (event === 'error') this.errorHandlers.push(cb as (e: Error) => void);
    else if (event === 'reconnecting') this.reconnectHandlers.push(cb as () => void);
    else if (event === 'connected') this.connectedHandlers.push(cb as (r: HandshakeReply) => void);
  }

  private _sendFrame(frame: Frame): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('NoiseSession: WebSocket not open');
    }
    const plaintext = cborEncode(frame);
    const ciphertext = this.sendCs!.encryptWithAd(new Uint8Array(0), plaintext);
    this._wsSend(this.ws, ciphertext);
  }

  private _wsSend(ws: WebSocket, data: Uint8Array): void {
    const msg = new Uint8Array(2 + data.length);
    msg[0] = (data.length >> 8) & 0xff;
    msg[1] = data.length & 0xff;
    msg.set(data, 2);
    ws.send(msg.buffer);
  }

  private _startHeartbeat(): void {
    this.pingTimer = setInterval(() => {
      try {
        const id = ++this.idCounter;
        this._sendFrame({ t: 'ping', id, ts: Date.now() });
        this.pongDeadline = setTimeout(() => {
          this.ws?.close();
        }, 60000);
      } catch {
        // Ignore — ws.onclose will handle reconnect
      }
    }, 30000);
  }

  private _stopHeartbeat(): void {
    if (this.pingTimer !== null) {
      clearInterval(this.pingTimer);
      this.pingTimer = null;
    }
    if (this.pongDeadline !== null) {
      clearTimeout(this.pongDeadline);
      this.pongDeadline = null;
    }
  }

  private _scheduleReconnect(
    resolveHandshake: (r: HandshakeReply) => void,
    rejectHandshake: (e: Error) => void,
  ): void {
    this.reconnectHandlers.forEach((h) => h());
    const delay = this.reconnectDelay;
    this.reconnectDelay = Math.min(this.reconnectDelay * 2, 30000);
    setTimeout(() => {
      if (!this.sessionClosed) {
        this.recvBuf = new Uint8Array(0);
        this._doConnect(resolveHandshake, rejectHandshake);
      }
    }, delay);
  }
}

// ── Utilities ────────────────────────────────────────────────────────────────

function concat(a: Uint8Array, b: Uint8Array): Uint8Array<ArrayBuffer> {
  const buf = new ArrayBuffer(a.length + b.length);
  const result = new Uint8Array(buf);
  result.set(a);
  result.set(b, a.length);
  return result;
}
