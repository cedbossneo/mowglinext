package providers

import (
	"context"
	"encoding/json"
	"log"
	"time"

	"github.com/cedbossneo/mowglinext/app/server/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/app/server/pkg/types"
)

type schedule struct {
	ID         string     `json:"id"`
	Area       int        `json:"area"`
	Time       string     `json:"time"`
	DaysOfWeek []int      `json:"daysOfWeek"`
	Enabled    bool       `json:"enabled"`
	CreatedAt  time.Time  `json:"createdAt"`
	LastRun    *time.Time `json:"lastRun,omitempty"`
}

const schedulerKeyPrefix = "schedule:"

type SchedulerProvider struct {
	rosProvider types.IRosProvider
	dbProvider  types.IDBProvider
}

func NewSchedulerProvider(rosProvider types.IRosProvider, dbProvider types.IDBProvider) *SchedulerProvider {
	s := &SchedulerProvider{
		rosProvider: rosProvider,
		dbProvider:  dbProvider,
	}
	go s.run()
	return s
}

func (s *SchedulerProvider) run() {
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	for range ticker.C {
		s.checkSchedules()
	}
}

func (s *SchedulerProvider) checkSchedules() {
	keys, err := s.dbProvider.KeysWithSuffix(schedulerKeyPrefix)
	if err != nil {
		return
	}

	now := time.Now()
	currentDay := int(now.Weekday())
	currentTime := now.Format("15:04")

	for _, key := range keys {
		data, err := s.dbProvider.Get(key)
		if err != nil {
			continue
		}

		var sched schedule
		if err := json.Unmarshal(data, &sched); err != nil {
			continue
		}

		if !sched.Enabled {
			continue
		}

		if !s.shouldRun(&sched, currentDay, currentTime, now) {
			continue
		}

		log.Printf("Scheduler: triggering mowing in area %d (schedule %s)", sched.Area, sched.ID)

		ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
		err = s.rosProvider.CallService(
			ctx,
			"/mowgli/behavior/start_in_area",
			&mowgli.StartInAreaReq{Area: uint8(sched.Area)},
			&mowgli.StartInAreaRes{},
		)
		cancel()

		if err != nil {
			log.Printf("Scheduler: failed to start mowing in area %d: %v", sched.Area, err)
			continue
		}

		// Update last run time
		sched.LastRun = &now
		if data, err := json.Marshal(&sched); err == nil {
			_ = s.dbProvider.Set(schedulerKeyPrefix+sched.ID, data)
		}
	}
}

func (s *SchedulerProvider) shouldRun(sched *schedule, currentDay int, currentTime string, now time.Time) bool {
	// Check if current time matches
	if sched.Time != currentTime {
		return false
	}

	// Check if current day is in schedule
	dayMatch := false
	for _, d := range sched.DaysOfWeek {
		if d == currentDay {
			dayMatch = true
			break
		}
	}
	if !dayMatch {
		return false
	}

	// Prevent double execution within the same minute
	if sched.LastRun != nil {
		if now.Sub(*sched.LastRun) < 2*time.Minute {
			return false
		}
	}

	return true
}
