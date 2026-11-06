import type { FrameHeader, Hello, State } from './types';

export interface Handlers {
  hello(h: Hello): void;
  state(s: State): void;
  frame(h: FrameHeader, jpeg: Blob): void;
  link(up: boolean): void;
}

/** 브리지 WebSocket. 끊기면 1 s 간격으로 다시 붙는다(부스에서 브리지를 재시작해도 화면은 그대로 둔다). */
export class Bridge {
  private ws: WebSocket | null = null;
  constructor(private h: Handlers) { this.open(); }

  private open() {
    const ws = new WebSocket(`ws://${location.host}/ws`);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => this.h.link(true);
    ws.onclose = () => { this.h.link(false); this.ws = null; setTimeout(() => this.open(), 1000); };
    ws.onerror = () => ws.close();
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') {
        const msg = JSON.parse(ev.data);
        if (msg.type === 'state') this.h.state(msg as State);
        else if (msg.type === 'hello') this.h.hello(msg as Hello);
        return;
      }
      const buf = ev.data as ArrayBuffer;
      const n = new DataView(buf).getUint32(0, true);
      const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, n))) as FrameHeader;
      this.h.frame(header, new Blob([new Uint8Array(buf, 4 + n)], { type: header.mime }));
    };
    this.ws = ws;
  }

  send(obj: unknown) {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }
}
