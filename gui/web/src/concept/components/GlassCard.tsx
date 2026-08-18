import type {CSSProperties, ReactNode} from "react";
import {motion, type Variants} from "framer-motion";
import {riseFade} from "../motion";
import {useThemeMode} from "../../theme/ThemeContext.tsx";

/**
 * Layered surface with the luminous edge gradient handled by `.glass` in
 * concept.css. Visual mode enables the glass blur; other modes preserve the
 * same edge, hierarchy, and contrast without re-filtering the backdrop.
 */

type Variant = "default" | "elevated" | "glow";

interface GlassCardProps {
  children: ReactNode;
  variant?: Variant;
  padding?: number | string;
  className?: string;
  style?: CSSProperties;
  /** Wrap in a motion.div with rise-fade entrance. */
  animate?: boolean;
  /** Custom variants override -- e.g. for staggered children. */
  motionVariants?: Variants;
  onClick?: () => void;
}

export function GlassCard({
  children,
  variant = "default",
  padding = 20,
  className = "",
  style,
  animate = false,
  motionVariants,
  onClick,
}: GlassCardProps) {
  const {colors, displayMode} = useThemeMode();
  const inner = (
    <div
      onClick={onClick}
      className={`glass ${className}`}
      style={{
        padding,
        background: variant === "elevated"
          ? colors.bgElevated
          : displayMode === 'efficient' ? colors.bgCard : colors.glassBackground,
        backdropFilter: displayMode === 'visual' ? 'blur(20px) saturate(135%)' : undefined,
        WebkitBackdropFilter: displayMode === 'visual' ? 'blur(20px) saturate(135%)' : undefined,
        boxShadow: variant === "glow"
          ? "var(--shadow-card), var(--shadow-glow-lime)"
          : "var(--shadow-card)",
        cursor: onClick ? "pointer" : undefined,
        ...style,
      }}
    >
      {children}
    </div>
  );

  if (!animate) return inner;
  return (
    <motion.div variants={motionVariants ?? riseFade}>
      {inner}
    </motion.div>
  );
}
