import { defineStore } from 'pinia';
import { useLocalStorage } from '@vueuse/core';
import Interaction from './types/Interaction';
import Job from './types/Job';

// How many interaction ids to remember. 
const NOTIFIED_LIMIT = 100;

// The fields of a job that notifyJobFinished() reads.
type FinishedJob = Pick<Job, 'id' | 'finished_at' | 'exit_status'>;

export const notificationsStore = defineStore( 'spiderpig-notifications',
	{
		state() {
			return {
				// The unfinished jobs that this user started.
				jobIds: useLocalStorage( 'spiderpig-job-ids', '[]' ),
				alreadyNotifiedInters: useLocalStorage( 'spiderpig-notified-interactions', '[]' ),
				// The open "needs you" notification of each job, by job id.  A
				// Notification does not survive a reload, so it is not stored.
				userNotifications: {} as Record<number, Notification>
			};
		},
		actions: {
			// Called by anything that initiates a new job.
			async setupUserNotificationsForJob( jobId: number ): Promise<void> {
				if ( Notification.permission === 'default' ) {
					await Notification.requestPermission();
				}
				if ( Notification.permission === 'granted' ) {
					this._trackJob( jobId );
				}
			},
			// LocalStorage values must be strings.
			// https://developer.mozilla.org/en-US/docs/Web/API/Window/localStorage#description
			_trackedJobIds(): number[] {
				return JSON.parse( this.jobIds );
			},
			_trackJob( jobId: number ): void {
				const tracked = this._trackedJobIds();
				if ( !tracked.includes( jobId ) ) {
					tracked.push( jobId );
					this.jobIds = JSON.stringify( tracked );
				}
			},
			_forgetJob( jobId: number ): void {
				this.jobIds = JSON.stringify(
					this._trackedJobIds().filter( ( id: number ) => id !== jobId )
				);
			},
			_jobMatchesTrackedJob( jobId: number ): boolean {
				return this._trackedJobIds().includes( jobId );
			},
			_userShouldBeNotified( interaction: Interaction ): boolean {
				return Notification.permission === 'granted' &&
				this._jobMatchesTrackedJob( interaction.job_id ) &&
				!this._alreadyNotified( interaction.id );
			},
			_jobFinishedBody( job: FinishedJob ): string {
				if ( job.exit_status === 0 ) {
					return `Job ${ job.id } finished successfully`;
				}

				return `Job ${ job.id } finished with errors`;
			},
			_alreadyNotified( interactionId: number ): boolean {
				return JSON.parse( this.alreadyNotifiedInters ).includes( interactionId );
			},
			_rememberNotified( interactionId: number ): void {
				const alreadyNotified = JSON.parse( this.alreadyNotifiedInters );
				alreadyNotified.push( interactionId );
				this.alreadyNotifiedInters = JSON.stringify(
					alreadyNotified.slice( -NOTIFIED_LIMIT )
				);
			},
			// notifyUser is called when an SpInteraction (Interaction.vue) component is mounted.
			notifyUser( interaction: Interaction ): void {
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
			notifyJobFinished( job: FinishedJob ): void {
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
			closeNotification( jobId: number ): void {
				const notification = this.userNotifications[ jobId ];
				if ( notification ) {
					notification.close();
					delete this.userNotifications[ jobId ];
				}
			}
		}
	}
);
