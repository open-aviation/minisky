import { reactive } from "vue";

export interface MiniskyAircraft {
  id: string;
  callsign: string;
  typecode?: string;
  latitude?: number;
  longitude?: number;
  /** feet */
  altitude?: number;
  /** knots */
  groundspeed?: number;
  /** knots */
  tas?: number;
  /** knots */
  ias?: number;
  /** feet per minute */
  vertical_rate?: number;
  /** degrees */
  track?: number;
  /** aircraft currently in a detected conflict */
  inconf: boolean;
  /** epoch seconds of the simulated UTC clock */
  timestamp?: number;
}

export interface SimInfo {
  simt: number;
  simdt: number;
  simutc?: string;
  speed: number;
  ntraf: number;
  state: number;
  state_name: string;
  scenname?: string;
  nconf_cur: number;
  nlos_cur: number;
}

export interface ConsoleEntry {
  kind: "cmd" | "msg" | "err";
  text: string;
}

export const miniskyStore = reactive({
  channel: "minisky",
  siminfo: null as SimInfo | null,
  connected: false,
  lastUpdate: 0,
  log: [] as ConsoleEntry[]
});

export function pushLog(kind: ConsoleEntry["kind"], text: string) {
  if (!text) return;
  miniskyStore.log.push({ kind, text });
  if (miniskyStore.log.length > 100) {
    miniskyStore.log.splice(0, miniskyStore.log.length - 100);
  }
}
