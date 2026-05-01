import React from "react";
import { QRCodeSVG } from "qrcode.react";

interface PairingQRProps {
    value: string;
    size: number;
}

/**
 * Thin wrapper around QRCodeSVG from qrcode.react.
 * Kept in its own file so OnboardingPage.tsx stays focused on wizard logic.
 */
const PairingQR: React.FC<PairingQRProps> = ({ value, size }) => {
    return (
        <QRCodeSVG
            value={value}
            size={size}
            level="M"
            marginSize={1}
        />
    );
};

export default PairingQR;
