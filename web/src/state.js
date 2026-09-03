import { defineStore } from 'pinia';
import { useLocalStorage } from '@vueuse/core';

// How many interaction ids to remember. 
const NOTIFIED_LIMIT = 100;

export const notificationsStore = defineStore( 'spiderpig-notifications',
	{
		state() {
			return {
				// The unfinished jobs that this user started.
				jobIds: useLocalStorage( 'spiderpig-job-ids', '[]' ),
				alreadyNotifiedInters: useLocalStorage( 'spiderpig-notified-interactions', '[]' ),
				// The open "needs you" notification of each job, by job id.  A
				// Notification does not survive a reload, so it is not stored.
				userNotifications: {}
			};
		},
		actions: {
			// Called by anything that initiates a new job.
			async setupUserNotificationsForJob( jobId ) {
				if ( Notification.permission === 'default' ) {
					await Notification.requestPermission();
				}
				if ( Notification.permission === 'granted' ) {
					this._trackJob( jobId );
				}
			},
			// LocalStorage values must be strings.
			// https://developer.mozilla.org/en-US/docs/Web/API/Window/localStorage#description
			_trackedJobIds() {
				return JSON.parse( this.jobIds );
			},
			_trackJob( jobId ) {
				const tracked = this._trackedJobIds();
				if ( !tracked.includes( jobId ) ) {
					tracked.push( jobId );
					this.jobIds = JSON.stringify( tracked );
				}
			},
			_forgetJob( jobId ) {
				this.jobIds = JSON.stringify(
					this._trackedJobIds().filter( ( id ) => id !== jobId )
				);
			},
			_jobMatchesTrackedJob( jobId ) {
				return this._trackedJobIds().includes( jobId );
			},
			_userShouldBeNotified( interaction ) {
				return Notification.permission === 'granted' &&
				this._jobMatchesTrackedJob( interaction.job_id ) &&
				!this._alreadyNotified( interaction.id );
			},
			_jobFinishedBody( job ) {
				if ( job.exit_status === 0 ) {
					return `Job ${ job.id } finished successfully`;
				}

				return `Job ${ job.id } finished with errors`;
			},
			_alreadyNotified( interactionId ) {
				return JSON.parse( this.alreadyNotifiedInters ).includes( interactionId );
			},
			_rememberNotified( interactionId ) {
				const alreadyNotified = JSON.parse( this.alreadyNotifiedInters );
				alreadyNotified.push( interactionId );
				this.alreadyNotifiedInters = JSON.stringify(
					alreadyNotified.slice( -NOTIFIED_LIMIT )
				);
			},
			// notifyUser is called when an SpInteraction (Interaction.vue) component is mounted.
			notifyUser( interaction ) {
				if (
					this._userShouldBeNotified( interaction ) &&
					// Keep in sync with the prompt message in scap/backport.py#Backport._do_backport
					!interaction.prompt.includes( 'Backport the changes?' )
				) {
					this.userNotifications[ interaction.job_id ] = new Notification(
						'SpiderPig needs you!',
						{
							body: `Job ${ interaction.job_id } requires user input`,
							requireInteraction: true
						}
					);
					this._rememberNotified( interaction.id );
				}
			},
			notifyJobFinished( job ) {
				if (
					Notification.permission !== 'granted' ||
					!job.finished_at ||
					!this._jobMatchesTrackedJob( job.id )
				) {
					return;
				}

				new Notification( 'SpiderPig job finished', {
					body: this._jobFinishedBody( job )
				} );
				this.closeNotification( job.id );
				this._forgetJob( job.id );
			},
			// Called from Interaction.vue when the user has responded.
			closeNotification( jobId ) {
				const notification = this.userNotifications[ jobId ];
				if ( notification ) {
					notification.close();
					delete this.userNotifications[ jobId ];
				}
			}
		}
	}
);
