import {App, Button, Card, Checkbox, Col, Row, Select, Switch, TimePicker, Table, Space} from "antd";
import {ClockCircleOutlined, DeleteOutlined, PlusOutlined, ExclamationCircleOutlined} from "@ant-design/icons";
import {useCallback, useEffect, useRef, useState} from "react";
import {useIsMobile} from "../hooks/useIsMobile";
import dayjs from "dayjs";
import {useApi, useWS, Map as MapType, useThemeMode} from "@mowglinext/common";

interface Schedule {
    id: string;
    area: number;
    time: string;
    daysOfWeek: number[];
    enabled: boolean;
    createdAt: string;
    lastRun?: string;
}

const DAYS = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"];
const DAY_LETTERS = ["S", "M", "T", "W", "T", "F", "S"];

function areaLabel(index: number, name: string | undefined): string {
    return name ? `${index + 1}. ${name}` : `Area ${index + 1}`;
}

export const SchedulePage = () => {
    const {colors} = useThemeMode();
    const guiApi = useApi();
    const {notification, modal} = App.useApp();
    const isMobile = useIsMobile();
    const [schedules, setSchedules] = useState<Schedule[]>([]);
    const [loading, setLoading] = useState(false);
    const [workingAreas, setWorkingAreas] = useState<Array<string | undefined>>([]);
    const fetchedRef = useRef(false);

    const mapStream = useWS<string>(
        () => { console.log("SchedulePage: map stream closed"); },
        () => { console.log("SchedulePage: map stream connected"); },
        (data) => {
            const parsed = JSON.parse(data) as MapType;
            const names = (parsed.WorkingArea ?? []).map((a) => a.Name);
            setWorkingAreas(names);
        },
    );

    useEffect(() => {
        mapStream.start("/api/openmower/subscribe/map");
        return () => { mapStream.stop(); };
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    const areaOptions = workingAreas.length > 0
        ? workingAreas.map((name, index) => ({
            label: areaLabel(index, name),
            value: index,
          }))
        : Array.from({length: Math.max(1, schedules.reduce((max, s) => Math.max(max, s.area + 1), 1))}, (_, i) => ({
            label: areaLabel(i, undefined),
            value: i,
          }));

    const fetchSchedules = useCallback(async () => {
        try {
            const response = await guiApi.request<{ schedules: Schedule[] }>({
                path: "/schedules",
                method: "GET",
                format: "json",
            });
            setSchedules(response.data.schedules ?? []);
        } catch {
            notification.error({message: "Failed to load schedules"});
        }
    }, [guiApi, notification]);

    useEffect(() => {
        if (fetchedRef.current) return;
        fetchedRef.current = true;
        fetchSchedules();
    }, [fetchSchedules]);

    const handleCreate = async () => {
        setLoading(true);
        try {
            await guiApi.request({
                path: "/schedules",
                method: "POST",
                body: {
                    area: 0,
                    time: "09:00",
                    daysOfWeek: [1, 2, 3, 4, 5],
                    enabled: false,
                },
                format: "json",
            });
            await fetchSchedules();
        } catch {
            notification.error({message: "Failed to create schedule"});
        } finally {
            setLoading(false);
        }
    };

    const handleUpdate = async (sched: Schedule) => {
        try {
            await guiApi.request({
                path: `/schedules/${sched.id}`,
                method: "PUT",
                body: sched,
                format: "json",
            });
            await fetchSchedules();
        } catch {
            notification.error({message: "Failed to update schedule"});
        }
    };

    const handleDelete = async (id: string) => {
        try {
            await guiApi.request({
                path: `/schedules/${id}`,
                method: "DELETE",
                format: "json",
            });
            await fetchSchedules();
        } catch {
            notification.error({message: "Failed to delete schedule"});
        }
    };

    const confirmDelete = (id: string) => {
        modal.confirm({
            title: "Delete schedule",
            icon: <ExclamationCircleOutlined/>,
            content: "Are you sure you want to delete this schedule?",
            okText: "Delete",
            okType: "danger",
            cancelText: "Cancel",
            onOk: () => handleDelete(id),
        });
    };

    const toggleDay = (sched: Schedule, day: number) => {
        const days = sched.daysOfWeek.includes(day)
            ? sched.daysOfWeek.filter(d => d !== day)
            : [...sched.daysOfWeek, day];
        handleUpdate({...sched, daysOfWeek: days});
    };

    // Mobile card-based layout — title comes from the page header, no duplicate
    if (isMobile) {
        return (
            <div style={{display: 'flex', flexDirection: 'column', gap: 12, paddingBottom: 8}}>
                <div style={{display: 'flex', justifyContent: 'flex-end'}}>
                    <Button
                        type="primary"
                        icon={<PlusOutlined/>}
                        onClick={handleCreate}
                        loading={loading}
                        size="large"
                        style={{borderRadius: 12, height: 44, paddingInline: 24}}
                    >
                        Add Schedule
                    </Button>
                </div>

                {schedules.length === 0 && (
                    <div style={{
                        textAlign: 'center',
                        padding: 32,
                        color: colors.textSecondary,
                        background: colors.bgCard,
                        borderRadius: 12,
                    }}>
                        No schedules configured yet.
                    </div>
                )}

                {schedules.map(sched => {
                    const optionsWithCurrent = areaOptions.some((o) => o.value === sched.area)
                        ? areaOptions
                        : [...areaOptions, {label: areaLabel(sched.area, undefined), value: sched.area}];

                    return (
                        <div key={sched.id} style={{
                            background: colors.bgCard,
                            borderRadius: 12,
                            padding: 16,
                            display: 'flex',
                            flexDirection: 'column',
                            gap: 12,
                        }}>
                            {/* Row 1: Switch + Area */}
                            <div style={{display: 'flex', alignItems: 'center', gap: 12}}>
                                <Switch
                                    checked={sched.enabled}
                                    onChange={(checked) => handleUpdate({...sched, enabled: checked})}
                                />
                                <Select
                                    value={sched.area}
                                    size="small"
                                    style={{flex: 1}}
                                    options={optionsWithCurrent}
                                    onChange={(val) => handleUpdate({...sched, area: val})}
                                />
                            </div>

                            {/* Row 2: Time picker */}
                            <TimePicker
                                value={dayjs(sched.time, "HH:mm")}
                                format="HH:mm"
                                onChange={(val) => {
                                    if (val) handleUpdate({...sched, time: val.format("HH:mm")});
                                }}
                                style={{width: '100%'}}
                                size="large"
                            />

                            {/* Row 3: Day circle toggles */}
                            <div style={{display: 'flex', gap: 6, justifyContent: 'space-between'}}>
                                {DAY_LETTERS.map((letter, i) => {
                                    const isActive = sched.daysOfWeek.includes(i);
                                    return (
                                        <button
                                            key={i}
                                            onClick={() => toggleDay(sched, i)}
                                            style={{
                                                width: 36,
                                                height: 36,
                                                borderRadius: '50%',
                                                border: `1.5px solid ${isActive ? colors.primary : colors.border}`,
                                                background: isActive ? colors.primaryBg : 'transparent',
                                                color: isActive ? colors.primary : colors.textSecondary,
                                                fontSize: 13,
                                                fontWeight: 600,
                                                cursor: 'pointer',
                                                transition: 'all 0.15s',
                                                padding: 0,
                                            }}
                                        >
                                            {letter}
                                        </button>
                                    );
                                })}
                            </div>

                            {/* Row 4: Last run + delete */}
                            <div style={{
                                display: 'flex',
                                justifyContent: 'space-between',
                                alignItems: 'center',
                                borderTop: `1px solid ${colors.borderSubtle}`,
                                paddingTop: 8,
                            }}>
                                <span style={{fontSize: 12, color: colors.textSecondary}}>
                                    Last: {sched.lastRun ? dayjs(sched.lastRun).format("YYYY-MM-DD HH:mm") : "Never"}
                                </span>
                                <Button
                                    type="text"
                                    danger
                                    icon={<DeleteOutlined/>}
                                    size="small"
                                    onClick={() => confirmDelete(sched.id)}
                                />
                            </div>
                        </div>
                    );
                })}
            </div>
        );
    }

    // Desktop table layout
    const columns = [
        {
            title: "Enabled",
            dataIndex: "enabled",
            key: "enabled",
            width: 80,
            render: (enabled: boolean, record: Schedule) => (
                <Switch
                    checked={enabled}
                    onChange={(checked) => handleUpdate({...record, enabled: checked})}
                />
            ),
        },
        {
            title: "Time",
            dataIndex: "time",
            key: "time",
            width: 120,
            render: (time: string, record: Schedule) => (
                <TimePicker
                    value={dayjs(time, "HH:mm")}
                    format="HH:mm"
                    onChange={(val) => {
                        if (val) handleUpdate({...record, time: val.format("HH:mm")});
                    }}
                    size="small"
                />
            ),
        },
        {
            title: "Area",
            dataIndex: "area",
            key: "area",
            width: 200,
            render: (area: number, record: Schedule) => {
                const optionsWithCurrent = areaOptions.some((o) => o.value === area)
                    ? areaOptions
                    : [...areaOptions, {label: areaLabel(area, undefined), value: area}];
                return (
                    <Select
                        value={area}
                        size="small"
                        style={{width: "100%"}}
                        options={optionsWithCurrent}
                        onChange={(val) => handleUpdate({...record, area: val})}
                    />
                );
            },
        },
        {
            title: "Days",
            dataIndex: "daysOfWeek",
            key: "daysOfWeek",
            render: (days: number[], record: Schedule) => (
                <Checkbox.Group
                    value={days}
                    onChange={(checked) => handleUpdate({...record, daysOfWeek: checked as number[]})}
                    options={DAYS.map((label, i) => ({label, value: i}))}
                />
            ),
        },
        {
            title: "Last Run",
            dataIndex: "lastRun",
            key: "lastRun",
            width: 160,
            render: (lastRun: string | undefined) =>
                lastRun ? dayjs(lastRun).format("YYYY-MM-DD HH:mm") : "Never",
        },
        {
            title: "",
            key: "actions",
            width: 50,
            render: (_: unknown, record: Schedule) => (
                <Button
                    type="text"
                    danger
                    icon={<DeleteOutlined/>}
                    size="small"
                    onClick={() => confirmDelete(record.id)}
                />
            ),
        },
    ];

    return (
        <Row gutter={[16, 16]}>
            <Col span={24}>
                <Card
                    title={<Space><ClockCircleOutlined/> Mowing Schedules</Space>}
                    size="small"
                    extra={
                        <Button
                            type="primary"
                            icon={<PlusOutlined/>}
                            onClick={handleCreate}
                            loading={loading}
                        >
                            Add Schedule
                        </Button>
                    }
                >
                    <Table
                        dataSource={schedules}
                        columns={columns}
                        rowKey="id"
                        pagination={false}
                        size="small"
                        scroll={{x: 600}}
                        locale={{emptyText: "No schedules configured. Click 'Add Schedule' to create one."}}
                    />
                </Card>
            </Col>
        </Row>
    );
};

export default SchedulePage;
