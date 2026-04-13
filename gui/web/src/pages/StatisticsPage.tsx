import {useCallback, useEffect, useState} from "react";
import {
    Card,
    Col,
    Progress,
    Row,
    Space,
    Statistic,
    Table,
    Tag,
    Typography,
} from "antd";
import {BarChartOutlined} from "@ant-design/icons";
import {useApi} from "../hooks/useApi.ts";
import {useDiagnosticsSnapshot} from "../hooks/useDiagnosticsSnapshot.ts";

// ── types ────────────────────────────────────────────────────────────────────

interface MowingSession {
    id: string;
    start_time: string;
    end_time: string;
    duration_seconds: number;
    area_name: string;
    coverage_percent: number;
    strips_completed: number;
    total_strips: number;
    distance_meters: number;
    status: "completed" | "aborted" | "error";
}

interface SessionsResponse {
    sessions: MowingSession[];
    total: number;
}

interface SessionStats {
    total_sessions: number;
    total_mowing_seconds: number;
    total_distance_meters: number;
    completed_sessions: number;
    average_coverage_percent: number;
}

// ── helpers ──────────────────────────────────────────────────────────────────

function formatDuration(seconds: number): string {
    if (!seconds || seconds <= 0) return "0m";
    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    if (h > 0) return `${h}h ${m}m`;
    return `${m}m`;
}

function formatTotalDuration(seconds: number): string {
    if (!seconds || seconds <= 0) return "0h 0m";
    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    return `${h}h ${m}m`;
}

function formatDistance(meters: number): string {
    if (!meters || meters <= 0) return "0 m";
    if (meters >= 1000) return `${(meters / 1000).toFixed(2)} km`;
    return `${Math.round(meters)} m`;
}

function formatDate(timestamp: string): string {
    if (!timestamp) return "--";
    return new Date(timestamp).toLocaleString(undefined, {
        year: "numeric",
        month: "short",
        day: "numeric",
        hour: "2-digit",
        minute: "2-digit",
    });
}

// ── main page ─────────────────────────────────────────────────────────────────

export const StatisticsPage = () => {
    const guiApi = useApi();
    const {snapshot} = useDiagnosticsSnapshot();

    const [sessions, setSessions] = useState<MowingSession[]>([]);
    const [stats, setStats] = useState<SessionStats | null>(null);
    const [loading, setLoading] = useState(false);

    const fetchData = useCallback(async () => {
        setLoading(true);
        try {
            const [sessionsRes, statsRes] = await Promise.all([
                guiApi.request<SessionsResponse>({
                    path: "/diagnostics/sessions",
                    method: "GET",
                    format: "json",
                }),
                guiApi.request<SessionStats>({
                    path: "/diagnostics/sessions/stats",
                    method: "GET",
                    format: "json",
                }),
            ]);
            setSessions(sessionsRes.data?.sessions ?? []);
            setStats(statsRes.data ?? null);
        } catch {
            // silently degrade — backend may not have sessions yet
        } finally {
            setLoading(false);
        }
    }, []);

    useEffect(() => {
        fetchData();
        const interval = setInterval(fetchData, 30000);
        return () => clearInterval(interval);
    }, [fetchData]);

    // ── Aggregate stats cards ────────────────────────────────────────────────

    const completionRate =
        stats && stats.total_sessions > 0
            ? Math.round((stats.completed_sessions / stats.total_sessions) * 100)
            : 0;

    const statsRow = (
        <Row gutter={[12, 12]}>
            <Col xs={12} sm={8} md={8} lg={4} xl={4}>
                <Card size="small">
                    <Statistic
                        title="Total Sessions"
                        value={stats?.total_sessions ?? 0}
                        prefix={<BarChartOutlined/>}
                    />
                </Card>
            </Col>
            <Col xs={12} sm={8} md={8} lg={5} xl={5}>
                <Card size="small">
                    <Statistic
                        title="Total Mowing Time"
                        value={formatTotalDuration(stats?.total_mowing_seconds ?? 0)}
                    />
                </Card>
            </Col>
            <Col xs={12} sm={8} md={8} lg={5} xl={5}>
                <Card size="small">
                    <Statistic
                        title="Total Distance"
                        value={formatDistance(stats?.total_distance_meters ?? 0)}
                    />
                </Card>
            </Col>
            <Col xs={12} sm={8} md={8} lg={5} xl={5}>
                <Card size="small">
                    <Statistic
                        title="Completion Rate"
                        value={completionRate}
                        suffix="%"
                        valueStyle={{
                            color: completionRate >= 80 ? undefined : completionRate >= 50 ? "#faad14" : "#ff4d4f",
                        }}
                    />
                </Card>
            </Col>
            <Col xs={12} sm={8} md={8} lg={5} xl={5}>
                <Card size="small">
                    <Statistic
                        title="Avg Coverage"
                        value={stats?.average_coverage_percent != null
                            ? Math.round(stats.average_coverage_percent * 100) / 100
                            : 0}
                        precision={1}
                        suffix="%"
                    />
                </Card>
            </Col>
        </Row>
    );

    // ── Session history table ────────────────────────────────────────────────

    const sessionColumns = [
        {
            title: "Date",
            dataIndex: "start_time",
            key: "start_time",
            sorter: (a: MowingSession, b: MowingSession) =>
                new Date(b.start_time).getTime() - new Date(a.start_time).getTime(),
            defaultSortOrder: "ascend" as const,
            render: (v: string) => (
                <Typography.Text style={{fontSize: 13}}>{formatDate(v)}</Typography.Text>
            ),
        },
        {
            title: "Duration",
            dataIndex: "duration_seconds",
            key: "duration_seconds",
            render: (v: number) => (
                <Typography.Text style={{fontSize: 13}}>{formatDuration(v)}</Typography.Text>
            ),
        },
        {
            title: "Area",
            dataIndex: "area_name",
            key: "area_name",
            render: (v: string) => (
                <Typography.Text style={{fontSize: 13}}>{v || "--"}</Typography.Text>
            ),
        },
        {
            title: "Coverage",
            dataIndex: "coverage_percent",
            key: "coverage_percent",
            render: (v: number) => (
                <Progress
                    percent={Math.round((v ?? 0) * 100) / 100}
                    size="small"
                    style={{minWidth: 80, maxWidth: 140}}
                />
            ),
        },
        {
            title: "Strips",
            key: "strips",
            render: (_: unknown, record: MowingSession) => (
                <Typography.Text style={{fontSize: 13}}>
                    {record.strips_completed ?? 0}
                    {record.total_strips ? ` / ${record.total_strips}` : ""}
                </Typography.Text>
            ),
        },
        {
            title: "Status",
            dataIndex: "status",
            key: "status",
            render: (v: string) => {
                const color =
                    v === "completed" ? "success" :
                    v === "aborted" ? "warning" : "error";
                return <Tag color={color}>{v ?? "--"}</Tag>;
            },
        },
    ];

    const sectionHistory = (
        <Card
            title="Session History"
            size="small"
        >
            <Table
                size="small"
                loading={loading}
                dataSource={sessions}
                columns={sessionColumns}
                rowKey="id"
                pagination={{pageSize: 20, showSizeChanger: false}}
                locale={{
                    emptyText: (
                        <Space direction="vertical" style={{padding: "24px 0"}}>
                            <Typography.Text type="secondary">
                                No mowing sessions recorded yet. Sessions are tracked automatically when the robot mows.
                            </Typography.Text>
                        </Space>
                    ),
                }}
            />
        </Card>
    );

    // ── Section 2: Coverage progress per area ───────────────────────────────

    const coverage = snapshot?.coverage ?? [];

    const sectionCoverage =
        coverage.length > 0 ? (
            <Card title="Area Coverage Progress" size="small">
                <Row gutter={[12, 12]}>
                    {coverage.map((area) => (
                        <Col key={area.area_index} xs={24} sm={12} lg={8}>
                            <Card size="small" style={{borderRadius: 8}}>
                                <Typography.Text
                                    strong
                                    style={{fontSize: 13, display: "block", marginBottom: 8}}
                                >
                                    Area {area.area_index}
                                </Typography.Text>
                                <Progress
                                    percent={Math.round(area.coverage_percent * 100) / 100}
                                    size="small"
                                    style={{marginBottom: 8}}
                                />
                                <Row gutter={[8, 4]}>
                                    <Col span={8}>
                                        <Statistic
                                            title="Total"
                                            value={area.total_cells}
                                            valueStyle={{fontSize: 14}}
                                        />
                                    </Col>
                                    <Col span={8}>
                                        <Statistic
                                            title="Mowed"
                                            value={area.mowed_cells}
                                            valueStyle={{fontSize: 14}}
                                        />
                                    </Col>
                                    <Col span={8}>
                                        <Statistic
                                            title="Strips left"
                                            value={area.strips_remaining}
                                            valueStyle={{fontSize: 14}}
                                        />
                                    </Col>
                                </Row>
                            </Card>
                        </Col>
                    ))}
                </Row>
            </Card>
        ) : null;

    // ── layout ───────────────────────────────────────────────────────────────

    return (
        <Row gutter={[12, 12]} style={{paddingBottom: 8}}>
            <Col span={24}>{statsRow}</Col>
            <Col span={24}>{sectionHistory}</Col>
            {sectionCoverage && <Col span={24}>{sectionCoverage}</Col>}
        </Row>
    );
};

export default StatisticsPage;
