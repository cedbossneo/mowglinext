import {Outlet, useMatches, useNavigate} from "react-router-dom";
import {Layout, Typography} from "antd";
import {
    BulbOutlined,
    ClockCircleOutlined,
    HeatMapOutlined,
    MessageOutlined,
    RobotOutlined,
    RocketOutlined,
    SettingOutlined,
    MenuOutlined,
    CloseOutlined,
} from '@ant-design/icons';
import {useCallback, useEffect, useState} from "react";
import {useIsMobile} from "../hooks/useIsMobile";
import {useIOSInstallPrompt} from "../hooks/useIOSInstallPrompt";
import {IOSInstallBanner} from "../components/IOSInstallBanner";
import {MowerStatus, useThemeMode} from "@mowglinext/common";

const navItems = [
    {key: '/openmower', label: 'Dashboard', icon: <RobotOutlined/>},
    {key: '/map', label: 'Map', icon: <HeatMapOutlined/>},
    {key: '/schedule', label: 'Schedule', icon: <ClockCircleOutlined/>},
    {key: '/onboarding', label: 'Onboarding', icon: <RocketOutlined/>},
    {key: '/settings', label: 'Settings', icon: <SettingOutlined/>},
    {key: '/logs', label: 'Logs', icon: <MessageOutlined/>},
];

const bottomNavItems = [
    {key: '/openmower', label: 'Home', icon: <RobotOutlined/>},
    {key: '/map', label: 'Map', icon: <HeatMapOutlined/>},
    {key: '/schedule', label: 'Schedule', icon: <ClockCircleOutlined/>},
    {key: '/settings', label: 'Settings', icon: <SettingOutlined/>},
];

const pageTitles: Record<string, string> = {
    '/openmower': 'Dashboard',
    '/onboarding': 'Onboarding',
    '/settings': 'Settings',
    '/map': 'Map',
    '/schedule': 'Schedule',
    '/logs': 'Logs',
};

export default function Root() {
    const {mode, toggleMode, colors} = useThemeMode();
    const route = useMatches();
    const navigate = useNavigate();
    const isMobile = useIsMobile();
    const [sidebarOpen, setSidebarOpen] = useState(false);
    const [railExpanded, setRailExpanded] = useState(false);
    const {showPrompt: showInstallPrompt, dismiss: dismissInstallPrompt} = useIOSInstallPrompt();

    // On first load, check if onboarding was completed. If not, redirect to onboarding.
    const [configChecked, setConfigChecked] = useState(false);
    useEffect(() => {
        if (configChecked) return;
        (async () => {
            try {
                const base = import.meta.env.DEV ? 'http://localhost:4006' : '';
                const res = await fetch(`${base}/api/settings/status`);
                const data = await res.json();
                if (!data.onboarding_completed && currentPath !== '/onboarding') {
                    navigate({pathname: '/onboarding'});
                }
            } catch { /* ignore — backend not reachable */ }
            setConfigChecked(true);
        })();
    }, [configChecked]);

    useEffect(() => {
        if (route.length === 1 && route[0].pathname === "/") {
            navigate({pathname: '/openmower'});
        }
    }, [route, navigate]);

    const currentPath = route.length > 1 ? route[1].pathname : '/openmower';
    const pageTitle = pageTitles[currentPath] ?? 'OpenMower';

    const handleNavigate = useCallback((key: string) => {
        navigate({pathname: key});
        setSidebarOpen(false);
    }, [navigate]);

    if (isMobile) {
        return (
            <div style={{
                display: 'flex',
                flexDirection: 'column',
                height: '100%',
                background: colors.bgBase,
                overflow: 'hidden',
            }}>
                {/* Mobile Header */}
                <header style={{
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'space-between',
                    padding: '0 12px',
                    paddingTop: 'env(safe-area-inset-top, 0px)',
                    background: colors.bgCard,
                    borderBottom: `1px solid ${colors.border}`,
                    minHeight: 48,
                    flexShrink: 0,
                    zIndex: 100,
                }}>
                    <div style={{display: 'flex', alignItems: 'center', gap: 8, minWidth: 0, flex: 1}}>
                        <button
                            onClick={() => setSidebarOpen(!sidebarOpen)}
                            aria-label={sidebarOpen ? 'Close menu' : 'Open menu'}
                            style={{
                                background: 'none', border: 'none', color: colors.text,
                                fontSize: 18, padding: 4, cursor: 'pointer', flexShrink: 0,
                            }}
                        >
                            {sidebarOpen ? <CloseOutlined/> : <MenuOutlined/>}
                        </button>
                        <Typography.Text strong style={{
                            fontSize: 16, color: colors.text,
                            whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis',
                        }}>
                            {pageTitle}
                        </Typography.Text>
                    </div>
                    <MowerStatus/>
                </header>

                {/* Mobile Slide-over Backdrop */}
                <div
                    onClick={() => setSidebarOpen(false)}
                    style={{
                        position: 'fixed', inset: 0, top: 48,
                        background: 'rgba(0,0,0,0.5)', zIndex: 199,
                        opacity: sidebarOpen ? 1 : 0,
                        pointerEvents: sidebarOpen ? 'auto' : 'none',
                        transition: 'opacity 0.25s ease-out',
                    }}
                />
                {/* Mobile Slide-over Nav */}
                <nav style={{
                    position: 'fixed', top: 48, left: 0, bottom: 56,
                    width: 260, background: colors.bgCard, zIndex: 200,
                    borderRight: `1px solid ${colors.border}`,
                    overflowY: 'auto',
                    transform: sidebarOpen ? 'translateX(0)' : 'translateX(-100%)',
                    transition: 'transform 0.25s ease-out',
                }}>
                    <div style={{padding: '8px 0'}}>
                        {navItems.map(item => {
                            const isActive = currentPath === item.key;
                            return (
                                <button
                                    key={item.key}
                                    onClick={() => handleNavigate(item.key)}
                                    style={{
                                        display: 'flex',
                                        alignItems: 'center',
                                        gap: 12,
                                        width: '100%',
                                        padding: '12px 20px',
                                        background: isActive ? colors.primaryBg : 'transparent',
                                        border: 'none',
                                        borderLeft: isActive ? `3px solid ${colors.primary}` : '3px solid transparent',
                                        color: isActive ? colors.primary : colors.text,
                                        fontSize: 15,
                                        cursor: 'pointer',
                                        transition: 'background 0.15s, color 0.15s',
                                    }}
                                >
                                    <span style={{fontSize: 18}}>{item.icon}</span>
                                    {item.label}
                                </button>
                            );
                        })}
                        <button
                            onClick={toggleMode}
                            aria-label="Toggle theme"
                            style={{
                                display: 'flex',
                                alignItems: 'center',
                                gap: 12,
                                width: '100%',
                                padding: '12px 20px',
                                background: 'none',
                                border: 'none',
                                borderLeft: '3px solid transparent',
                                color: colors.text,
                                fontSize: 15,
                                cursor: 'pointer',
                            }}
                        >
                            <span style={{fontSize: 18}}><BulbOutlined/></span>
                            {mode === 'dark' ? 'Light' : 'Dark'}
                        </button>
                    </div>
                </nav>

                {/* Mobile Content */}
                <main style={{flex: 1, overflow: 'auto', padding: '8px 8px 0', minHeight: 0}}>
                    <Outlet/>
                </main>

                {/* Mobile Bottom Tab Bar */}
                <nav style={{
                    display: 'flex',
                    alignItems: 'stretch',
                    background: colors.bgCard,
                    borderTop: `1px solid ${colors.border}`,
                    minHeight: 56,
                    paddingBottom: 'env(safe-area-inset-bottom, 0px)',
                    flexShrink: 0,
                    zIndex: 100,
                }}>
                    {bottomNavItems.map(item => {
                        const isActive = currentPath === item.key;
                        return (
                            <button
                                key={item.key}
                                onClick={() => handleNavigate(item.key)}
                                aria-label={item.label}
                                aria-current={isActive ? 'page' : undefined}
                                style={{
                                    flex: 1,
                                    display: 'flex',
                                    flexDirection: 'column',
                                    alignItems: 'center',
                                    justifyContent: 'center',
                                    gap: 2,
                                    background: 'none',
                                    border: 'none',
                                    cursor: 'pointer',
                                    color: isActive ? colors.primary : colors.muted,
                                    fontSize: 20,
                                    borderTop: isActive ? `2px solid ${colors.primary}` : '2px solid transparent',
                                    transition: 'color 0.2s, border-color 0.2s',
                                    padding: 0,
                                }}
                            >
                                {item.icon}
                                <span style={{fontSize: 10, lineHeight: 1}}>{item.label}</span>
                            </button>
                        );
                    })}
                </nav>
                {showInstallPrompt && <IOSInstallBanner onDismiss={dismissInstallPrompt}/>}
            </div>
        );
    }

    // Desktop layout — custom icon rail
    return (
        <div style={{display: 'flex', height: '100%', minHeight: '100%', maxHeight: '100%', overflow: 'hidden'}}>
            <nav
                onMouseEnter={() => setRailExpanded(true)}
                onMouseLeave={() => setRailExpanded(false)}
                style={{
                    width: railExpanded ? 200 : 60,
                    minWidth: railExpanded ? 200 : 60,
                    background: colors.bgCard,
                    borderRight: `1px solid ${colors.border}`,
                    display: 'flex',
                    flexDirection: 'column',
                    transition: 'width 0.2s ease, min-width 0.2s ease',
                    overflow: 'hidden',
                    flexShrink: 0,
                    height: '100%',
                }}
            >
                {/* Logo area */}
                <div style={{
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: railExpanded ? 'flex-start' : 'center',
                    gap: 10,
                    padding: railExpanded ? '20px 16px' : '20px 0',
                    borderBottom: `1px solid ${colors.borderSubtle}`,
                    overflow: 'hidden',
                    height: 64,
                    flexShrink: 0,
                }}>
                    <img src={mode === 'dark' ? '/logo-square.svg' : '/logo-square-dark.svg'} alt="OpenMower" style={{width: 24, height: 24, flexShrink: 0}} />
                    {railExpanded && (
                        <Typography.Text strong style={{
                            fontSize: 18, color: colors.text, whiteSpace: 'nowrap',
                        }}>
                            OpenMower
                        </Typography.Text>
                    )}
                </div>

                {/* Nav items */}
                <div style={{flex: 1, padding: '8px 0', overflowY: 'auto'}}>
                    {navItems.map(item => {
                        const isActive = currentPath === item.key;
                        return (
                            <button
                                key={item.key}
                                onClick={() => handleNavigate(item.key)}
                                aria-label={item.label}
                                style={{
                                    display: 'flex',
                                    alignItems: 'center',
                                    gap: 12,
                                    width: '100%',
                                    padding: '10px 0',
                                    paddingLeft: railExpanded ? 16 : 0,
                                    justifyContent: railExpanded ? 'flex-start' : 'center',
                                    background: isActive ? colors.primaryBg : 'transparent',
                                    border: 'none',
                                    borderLeft: isActive ? `3px solid ${colors.primary}` : '3px solid transparent',
                                    color: isActive ? colors.primary : colors.text,
                                    fontSize: 14,
                                    cursor: 'pointer',
                                    transition: 'background 0.15s, color 0.15s, padding-left 0.2s ease',
                                    overflow: 'hidden',
                                    whiteSpace: 'nowrap',
                                }}
                                onMouseOver={(e) => {
                                    if (!isActive) e.currentTarget.style.background = colors.bgElevated;
                                }}
                                onMouseOut={(e) => {
                                    if (!isActive) e.currentTarget.style.background = 'transparent';
                                }}
                            >
                                <span style={{fontSize: 22, flexShrink: 0, width: 28, textAlign: 'center', display: 'inline-flex', alignItems: 'center', justifyContent: 'center'}}>{item.icon}</span>
                                {railExpanded && (
                                    <span>{item.label}</span>
                                )}
                            </button>
                        );
                    })}
                </div>

                {/* Theme toggle */}
                <div style={{padding: '8px 0', borderTop: `1px solid ${colors.borderSubtle}`, flexShrink: 0}}>
                    <button
                        onClick={toggleMode}
                        aria-label="Toggle theme"
                        style={{
                            display: 'flex',
                            alignItems: 'center',
                            gap: 12,
                            width: '100%',
                            padding: '10px 0',
                            paddingLeft: railExpanded ? 16 : 0,
                            justifyContent: railExpanded ? 'flex-start' : 'center',
                            background: 'none',
                            border: 'none',
                            cursor: 'pointer',
                            color: colors.text,
                            fontSize: 14,
                            overflow: 'hidden',
                            whiteSpace: 'nowrap',
                        }}
                    >
                        <span style={{fontSize: 22, flexShrink: 0, width: 28, textAlign: 'center', display: 'inline-flex', alignItems: 'center', justifyContent: 'center'}}>
                            <BulbOutlined/>
                        </span>
                        {railExpanded && (
                            <span>{mode === 'dark' ? 'Light' : 'Dark'}</span>
                        )}
                    </button>
                </div>
            </nav>

            {/* Main content area */}
            <div style={{flex: 1, display: 'flex', flexDirection: 'column', height: '100%', overflow: 'hidden', minWidth: 0}}>
                <Layout.Header style={{
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'space-between',
                    gap: 12,
                    padding: '0 24px',
                    background: colors.bgCard,
                    borderBottom: `1px solid ${colors.border}`,
                    height: 48,
                    lineHeight: '48px',
                    overflow: 'hidden',
                    flexShrink: 0,
                }}>
                    <Typography.Text strong style={{
                        fontSize: 16, color: colors.text,
                        whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis',
                        flexShrink: 1, minWidth: 0,
                    }}>
                        {pageTitle}
                    </Typography.Text>
                    <MowerStatus/>
                </Layout.Header>
                <main style={{flex: 1, padding: '10px 24px 0 24px', overflow: 'auto', minHeight: 0}}>
                    <Outlet/>
                </main>
            </div>
        </div>
    );
}
