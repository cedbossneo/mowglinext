import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {NewAreaModal} from './NewAreaModal';

describe('NewAreaModal', () => {
    const defaultProps = {
        open: true,
        areaType: 'workarea' as const,
        areaName: '',
        onAreaTypeChange: vi.fn(),
        onAreaNameChange: vi.fn(),
        onSave: vi.fn(),
        onCancel: vi.fn(),
    };

    it('renders when open', () => {
        render(<NewAreaModal {...defaultProps} />);
        expect(screen.getByText('New area')).toBeInTheDocument();
    });

    it('does not render content when closed', () => {
        render(<NewAreaModal {...defaultProps} open={false} />);
        expect(screen.queryByText('New area')).not.toBeInTheDocument();
    });

    it('shows name input for workarea type', () => {
        render(<NewAreaModal {...defaultProps} areaType="workarea" />);
        expect(screen.getByPlaceholderText('e.g. Front lawn')).toBeInTheDocument();
    });

    it('hides name input for navigation type', () => {
        render(<NewAreaModal {...defaultProps} areaType="navigation" />);
        expect(screen.queryByPlaceholderText('e.g. Front lawn')).not.toBeInTheDocument();
    });

    it('hides name input for obstacle type', () => {
        render(<NewAreaModal {...defaultProps} areaType="obstacle" />);
        expect(screen.queryByPlaceholderText('e.g. Front lawn')).not.toBeInTheDocument();
    });

    it('calls onSave when Add area clicked', async () => {
        const onSave = vi.fn();
        const user = userEvent.setup();
        render(<NewAreaModal {...defaultProps} onSave={onSave} />);
        await user.click(screen.getByText('Add area'));
        expect(onSave).toHaveBeenCalled();
    });

    it('calls onCancel when Cancel clicked', async () => {
        const onCancel = vi.fn();
        const user = userEvent.setup();
        render(<NewAreaModal {...defaultProps} onCancel={onCancel} />);
        await user.click(screen.getByText('Cancel'));
        expect(onCancel).toHaveBeenCalled();
    });

    it('shows area type selector', () => {
        render(<NewAreaModal {...defaultProps} />);
        expect(screen.getByText('Working Area')).toBeInTheDocument();
    });
});
