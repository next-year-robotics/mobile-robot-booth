import { Conversation, type VoiceConversation, type PartialOptions } from '@elevenlabs/client';
import { PERSONA } from './persona';

const startVoice = (options: PartialOptions) => Conversation.startSession({ ...options, textOnly: false });

const ERRORS: Record<string, string> = {
  no_agent: 'ElevenLabs Agent ID를 config/booth.yaml에 설정해 주세요.',
  no_key: 'ElevenLabs API 키 파일이 없습니다. 자기소개는 바로 재생할 수 있어요.',
  disabled: '실시간 대화가 꺼져 있습니다.', bridge_offline: '브리지 연결을 확인해 주세요.',
  auth_error: 'ElevenLabs 키와 Agent 접근 권한을 확인해 주세요.',
  network_error: '인터넷 연결을 확인해 주세요.', timeout: '연결 시간이 초과되었습니다.',
  rate_limited: '잠시 뒤 다시 눌러 주세요.',
};
export class LiveVoice {
  private session?: VoiceConversation;
  private connecting = false;
  private generation = 0;
  private held = false;
  private idle = 0;
  private limit = 0;
  private started = 0;
  private retryAt = 0;
  private failures = 0;
  constructor(private token: () => Promise<string>,
    private status: (state: string, detail?: string) => void,
    private message: (text: string) => void,
    private tools: Record<string, (args: unknown) => string>,
    private start: typeof startVoice = startVoice,
    private event: (phase: 'start' | 'end', duration: number) => void = () => {}) {}
  level() { return this.held ? 0 : Math.min(1, (this.session?.getOutputVolume() ?? 0) * 3); }
  async press() {
    if (this.held) return;
    this.held = true; clearTimeout(this.idle);
    this.limit = window.setTimeout(() => this.release(), 30000);
    if (this.session) { this.session.setVolume({ volume: 0 }); this.session.sendUserActivity(); this.session.setMicMuted(false); this.status('듣는 중'); return; }
    if (this.connecting) return;
    if (Date.now() < this.retryAt) { this.release(); this.status('대화 불가', '잠시 뒤 V를 다시 눌러 주세요.'); return; }
    this.connecting = true;
    const generation = ++this.generation;
    this.status('연결 중', '연결 후 “듣는 중”이 되면 말씀해 주세요.');
    try {
      const signedUrl = await this.token();
      if (generation !== this.generation) return;
      const session = await this.start({
        signedUrl, connectionType: 'websocket', textOnly: false,
        overrides: { agent: { prompt: { prompt: PERSONA }, language: 'ko', firstMessage: '' } },
        clientTools: this.tools,
        onConversationCreated: c => c.setMicMuted(!this.held || generation !== this.generation),
        onMessage: m => { if (generation === this.generation) this.message(`${m.source === 'user' ? '관람객' : '막내'} · ${m.message}`); },
        onModeChange: ({ mode }) => {
          if (generation !== this.generation) return;
          if (!this.held) this.status(mode === 'speaking' ? '말하는 중' : '대기');
          if (mode === 'speaking') clearTimeout(this.idle);
          else if (!this.held) this.armIdle();
        },
        onDisconnect: () => {
          if (generation !== this.generation) return;
          this.finishMetrics(); this.session = undefined; this.held = false; clearTimeout(this.limit); clearTimeout(this.idle);
          this.status('대기', '대화가 종료되었습니다. V를 누르면 새로 연결합니다.'); this.message('');
        },
        onError: () => { if (generation === this.generation) { void this.stop(); this.status('대화 불가', 'ElevenLabs 연결 또는 Agent 설정을 확인해 주세요.'); } },
        onMCPToolApprovalRequest: () => false,
      });
      if (generation !== this.generation) { await session.endSession(); return; }
      this.session = session; this.started = performance.now(); this.event('start', 0); this.failures = 0;
      session.setMicMuted(!this.held);
      this.status(this.held ? '듣는 중' : '대기');
      if (!this.held) this.armIdle();
    } catch (e) {
      if (generation !== this.generation) return;
      this.held = false; clearTimeout(this.limit);
      this.retryAt = Date.now() + Math.min(30000, 1000 * 2 ** this.failures++);
      const key = e instanceof Error ? e.message : '';
      this.status('대화 불가', e instanceof DOMException && e.name === 'NotAllowedError'
        ? '브라우저 마이크 권한을 허용해 주세요.' : ERRORS[key] ?? '마이크·인터넷·ElevenLabs Agent 설정을 확인해 주세요.');
    } finally { if (generation === this.generation) this.connecting = false; }
  }
  release() {
    if (!this.held) return;
    this.held = false; clearTimeout(this.limit);
    this.session?.setMicMuted(true); this.session?.setVolume({ volume: 1 });
    if (this.session) { this.status('생각 중'); this.armIdle(); }
  }
  private armIdle() { clearTimeout(this.idle); this.idle = window.setTimeout(() => { void this.stop(); }, 60000); }
  async stop() {
    const generation = ++this.generation; this.held = false; this.connecting = false;
    clearTimeout(this.idle); clearTimeout(this.limit);
    const session = this.session; this.session = undefined; this.finishMetrics();
    if (session) { session.setMicMuted(true); await session.endSession().catch(() => {}); }
    if (generation === this.generation) { this.started = 0; this.message(''); this.status('대기'); }
  }
  private finishMetrics() { if (this.started) { this.event('end', this.duration); this.started = 0; } }
  get duration() { return this.started ? (performance.now() - this.started) / 1000 : 0; }
}
