export interface TimelineRange { from: number; to: number }

/** Both endpoints are chosen with Ctrl+click, independently of the playhead. */
export class TimelineSelection {
  private anchor = -1;
  private end = -1;

  get range(): TimelineRange | null {
    return this.anchor < 0 ? null : { from: Math.min(this.anchor, this.end), to: Math.max(this.anchor, this.end) };
  }

  selectEndpoint(index: number): void {
    if (this.anchor < 0) this.anchor = index;
    this.end = index;
  }

  setRange(from: number, to: number): void { this.anchor = from; this.end = to; }
  clear(): void { this.anchor = this.end = -1; }
}
