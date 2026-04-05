import {Form, Input, Modal, Select} from "antd";

interface NewAreaModalProps {
    open: boolean;
    areaType: 'workarea' | 'navigation' | 'obstacle';
    areaName: string;
    onAreaTypeChange: (v: 'workarea' | 'navigation' | 'obstacle') => void;
    onAreaNameChange: (v: string) => void;
    onSave: () => void;
    onCancel: () => void;
}

export const NewAreaModal = ({open, areaType, areaName, onAreaTypeChange, onAreaNameChange, onSave, onCancel}: NewAreaModalProps) => (
    <Modal
        open={open}
        title="New area"
        okText="Add area"
        cancelText="Cancel"
        onOk={onSave}
        onCancel={onCancel}
        destroyOnClose
    >
        <Form layout="vertical" style={{marginTop: 16}}>
            <Form.Item label="Area type">
                <Select
                    value={areaType}
                    onChange={onAreaTypeChange}
                    options={[
                        {value: 'workarea', label: 'Working Area'},
                        {value: 'navigation', label: 'Navigation Area'},
                        {value: 'obstacle', label: 'Obstacle'},
                    ]}
                />
            </Form.Item>
            {areaType === 'workarea' && (
                <Form.Item label="Area name (optional)">
                    <Input
                        placeholder="e.g. Front lawn"
                        value={areaName}
                        onChange={(e) => onAreaNameChange(e.target.value)}
                        onPressEnter={onSave}
                        autoFocus
                    />
                </Form.Item>
            )}
        </Form>
    </Modal>
);
