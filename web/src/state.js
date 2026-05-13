import { defineStore } from 'pinia';
import { useLocalStorage } from '@vueuse/core';

export const notificationsStore = defineStore( 'spiderpig-notifications',
	{
		state() {
			return {
				jobId: useLocalStorage( 'spiderpig-job-id', null ),
				alreadyNotifiedInters: useLocalStorage( 'spiderpig-notified-interactions', '[]' ),
				userNotification: null
			};
		},
		actions: {
			// Called by anything that initiates a new job.
			async setupUserNotificationsForJob( jobId ) {
				this._reset();
				if ( Notification.permission === 'default' ) {
					await Notification.requestPermission();
				}
				if ( Notification.permission === 'granted' ) {
					// LocalStorage values must be strings.
					// https://developer.mozilla.org/en-US/docs/Web/API/Window/localStorage#description
					this.jobId = JSON.stringify( jobId );
				}
			},
			_userShouldBeNotified( interaction ) {
				return Notification.permission === 'granted' &&
				interaction.job_id === JSON.parse( this.jobId ) &&
				!this._alreadyNotified( interaction.id );
			},
			_jobMatchesTrackedJob( jobId ) {
				if ( !this.jobId ) {
					return false;
				}

				return jobId === JSON.parse( this.jobId );
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
				this.alreadyNotifiedInters = JSON.stringify( alreadyNotified );
			},
			// notifyUser is called when an SpInteraction (Interaction.vue) component is mounted.
			notifyUser( interaction ) {
				if (
					this._userShouldBeNotified( interaction ) &&
					// Keep in sync with the prompt message in scap/backport.py#Backport._do_backport
					!interaction.prompt.includes( 'Backport the changes?' )
				) {
					this.userNotification = new Notification( 'SpiderPig needs you!', {
						body: `Job ${ interaction.job_id } requires user input`,
						requireInteraction: true
					} );
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
				this.jobId = null;
			},
			// Called from Interaction.vue when the user has responded.
			closeNotification() {
				if ( this.userNotification ) {
					this.userNotification.close();
					this.userNotification = null;
				}
			},
			_reset() {
				this.$reset();
				this.jobId = null;
				this.alreadyNotifiedInters = '[]';
				this.userNotification = null;
			}
		}
	}
);
