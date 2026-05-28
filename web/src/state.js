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
			async notifyJobFinished( job ) {
				if ( Notification.permission !== 'granted' || !job.finished_at ) {
					return;
				}

				await navigator.locks.request( 'spiderpig-notify-finished', () => {
					// Read directly from localStorage rather than the reactive
					// ref so we observe writes from sibling tabs that may not
					// have propagated through the storage event yet.
					const stored = localStorage.getItem( 'spiderpig-job-id' );
					if ( !stored || JSON.parse( stored ) !== job.id ) {
						return;
					}

					// Clear before firing so any tab queued behind us on the
					// lock sees null and bails.
					this.jobId = null;

					new Notification( 'SpiderPig job finished', {
						body: this._jobFinishedBody( job )
					} );
				} );
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
