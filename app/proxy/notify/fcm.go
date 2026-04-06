package notify

import (
	"context"
	"log"
)

// FCMDispatcher sends push notifications via Firebase Cloud Messaging
type FCMDispatcher struct {
	credentials string
}

func NewFCMDispatcher(credentials string) *FCMDispatcher {
	return &FCMDispatcher{credentials: credentials}
}

func (f *FCMDispatcher) Send(ctx context.Context, event *NotificationEvent, tokens []string) error {
	if f.credentials == "" {
		log.Printf("FCM not configured, skipping notification: %s - %s", event.Title, event.Body)
		return nil
	}

	// TODO: implement FCM HTTP v1 API
	// POST https://fcm.googleapis.com/v1/projects/{project}/messages:send
	// For each token, send:
	// {
	//   "message": {
	//     "token": "<device_token>",
	//     "notification": { "title": event.Title, "body": event.Body },
	//     "data": event.Data
	//   }
	// }
	log.Printf("Would send FCM notification to %d devices: %s - %s", len(tokens), event.Title, event.Body)
	return nil
}
