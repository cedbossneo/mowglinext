import type {SVGProps} from "react";

type IconProps = SVGProps<SVGSVGElement> & { size?: number };

const defaults = (p: IconProps, fill = "none") => ({
  viewBox: "0 0 24 24",
  width: p.size ?? 18,
  height: p.size ?? 18,
  fill,
  stroke: fill === "none" ? "currentColor" : undefined,
  strokeWidth: fill === "none" ? 1.8 : undefined,
  strokeLinecap: "round" as const,
  strokeLinejoin: "round" as const,
  ...p,
  size: undefined,
});

export const IconMower = (p: IconProps) => (
  <svg {...defaults(p)}>
    <rect x="3" y="9" width="18" height="8" rx="2"/>
    <path d="M7 9V6h10v3"/>
    <circle cx="7.5" cy="17.5" r="1.5"/>
    <circle cx="16.5" cy="17.5" r="1.5"/>
    <path d="M12 13.5v-1"/>
  </svg>
);

export const IconHome = (p: IconProps) => (
  <svg {...defaults(p)}>
    <path d="M3 11l9-7 9 7"/>
    <path d="M5 10v10h14V10"/>
  </svg>
);

export const IconPlay = (p: IconProps) => (
  <svg {...defaults(p, "currentColor")}><path d="M7 5v14l12-7z"/></svg>
);

export const IconPause = (p: IconProps) => (
  <svg {...defaults(p, "currentColor")}>
    <rect x="6" y="5" width="4" height="14"/>
    <rect x="14" y="5" width="4" height="14"/>
  </svg>
);

export const IconAlert = (p: IconProps) => (
  <svg {...defaults(p)}>
    <path d="M12 3L2 20h20L12 3z"/>
    <path d="M12 10v5M12 17.5v.5"/>
  </svg>
);

export const IconBolt = (p: IconProps) => (
  <svg {...defaults(p, "currentColor")}><path d="M13 2L4 14h6l-1 8 9-12h-6l1-8z"/></svg>
);

export const IconBattery = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <rect x="2" y="7" width="18" height="10" rx="2"/>
    <rect x="21" y="10" width="2" height="4" rx="0.5" fill="currentColor"/>
  </svg>
);

export const IconSignal = (p: IconProps) => (
  <svg {...defaults(p)}>
    <path d="M3 12a12 12 0 0118 0"/>
    <path d="M6 15a8 8 0 0112 0"/>
    <path d="M9 18a4 4 0 016 0"/>
    <circle cx="12" cy="20.5" r="0.8" fill="currentColor" stroke="none"/>
  </svg>
);

export const IconThermo = (p: IconProps) => (
  <svg {...defaults(p)}>
    <path d="M10 14V5a2 2 0 014 0v9a4 4 0 11-4 0z"/>
  </svg>
);

export const IconCpu = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <rect x="6" y="6" width="12" height="12" rx="2"/>
    <path d="M9 3v3M12 3v3M15 3v3M9 18v3M12 18v3M15 18v3M3 9h3M3 12h3M3 15h3M18 9h3M18 12h3M18 15h3"/>
  </svg>
);

export const IconRain = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <path d="M7 15a5 5 0 010-10 6 6 0 0111 2 4 4 0 012 8"/>
    <path d="M8 19l-1 2M12 19l-1 2M16 19l-1 2"/>
  </svg>
);

export const IconGear = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <circle cx="12" cy="12" r="3"/>
    <path d="M19 12a7 7 0 00-.1-1.2l2-1.5-2-3.4-2.3.9a7 7 0 00-2.1-1.2L14 3h-4l-.5 2.6a7 7 0 00-2.1 1.2l-2.3-.9-2 3.4 2 1.5A7 7 0 005 12c0 .4 0 .8.1 1.2l-2 1.5 2 3.4 2.3-.9a7 7 0 002.1 1.2L10 21h4l.5-2.6a7 7 0 002.1-1.2l2.3.9 2-3.4-2-1.5c.1-.4.1-.8.1-1.2z"/>
  </svg>
);

export const IconChart = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <path d="M4 20h16"/>
    <path d="M6 16l3-4 3 3 6-7"/>
  </svg>
);

export const IconClock = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <circle cx="12" cy="12" r="9"/>
    <path d="M12 7v5l3 2"/>
  </svg>
);

export const IconMap = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <path d="M9 3L3 5v16l6-2 6 2 6-2V3l-6 2z"/>
    <path d="M9 3v16M15 5v16"/>
  </svg>
);

export const IconSchedule = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <rect x="3" y="5" width="18" height="16" rx="2"/>
    <path d="M3 10h18M8 3v4M16 3v4"/>
  </svg>
);

export const IconLogs = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <path d="M21 11.5a8.5 8.5 0 11-3-6.5"/>
    <path d="M21 3v5h-5"/>
  </svg>
);

export const IconStats = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <path d="M4 20V10M10 20V4M16 20v-8M22 20h-20"/>
  </svg>
);

export const IconDiag = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <path d="M12 2v4M12 18v4M4.93 4.93l2.83 2.83M16.24 16.24l2.83 2.83M2 12h4M18 12h4M4.93 19.07l2.83-2.83M16.24 7.76l2.83-2.83"/>
    <circle cx="12" cy="12" r="3"/>
  </svg>
);

export const IconRocket = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <path d="M14 4c5 0 6 6 6 6s-6-1-6-6z"/>
    <path d="M13 11L9 7 4 9l3 3-2 5 5-2 3 3 2-5z"/>
    <circle cx="15" cy="9" r="1"/>
  </svg>
);

export const IconBulb = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <path d="M9 18h6M10 21h4M12 3a6 6 0 00-4 10.5c1 1 1.5 1.8 1.5 3.5h5c0-1.7.5-2.5 1.5-3.5A6 6 0 0012 3z"/>
  </svg>
);

export const IconBlades = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 1.6}}>
    <circle cx="12" cy="12" r="2"/>
    <path d="M12 10c-3-2-6-2-8 0M12 14c3 2 6 2 8 0M10 12c-2 3-2 6 0 8M14 12c2-3 2-6 0-8"/>
  </svg>
);

export const IconPlus = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 2}}>
    <path d="M12 5v14M5 12h14"/>
  </svg>
);

export const IconChevron = (p: IconProps) => (
  <svg {...{...defaults(p), strokeWidth: 2, viewBox: "0 0 24 24", width: p.size ?? 16, height: p.size ?? 16}}>
    <path d="M9 18l6-6-6-6"/>
  </svg>
);
