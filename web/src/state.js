import { defineStore } from 'pinia';
import { useLocalStorage } from '@vueuse/core';

export const notificationsStore = defineStore( 'spiderpig-notifications',
	{
		state() {
			return {
				backportJob: useLocalStorage( 'spiderpig-bp-job', null ),
				alreadyNotifiedInters: useLocalStorage( 'spiderpig-notified-interactions', '[]' ),
				userNotification: null
			};
		},
		actions: {
			async setUserNotifications( jobId ) {
				this.reset();
				if ( Notification.permission === 'default' ) {
					await Notification.requestPermission();
				}
				if ( Notification.permission === 'granted' ) {
					// LocalStorage values must be strings.
					// https://developer.mozilla.org/en-US/docs/Web/API/Window/localStorage#description
					this.backportJob = JSON.stringify( jobId );
				}
			},
			userShouldBeNotified( interaction ) {
				return Notification.permission === 'granted' &&
				interaction.job_id === JSON.parse( this.backportJob ) &&
				!this.alreadyNotified( interaction.id );
			},
			alreadyNotified( interactionId ) {
				return JSON.parse( this.alreadyNotifiedInters ).includes( interactionId );
			},
			rememberNotified( interactionId ) {
				const alreadyNotified = JSON.parse( this.alreadyNotifiedInters );
				alreadyNotified.push( interactionId );
				this.alreadyNotifiedInters = JSON.stringify( alreadyNotified );
			},
			notifyUser( interaction ) {
				if (
					this.userShouldBeNotified( interaction ) &&
					// Keep in sync with the prompt message in scap/backport.py#Backport._do_backport
					!interaction.prompt.includes( 'Backport the changes?' )
				) {
					this.userNotification = new Notification( 'SpiderPig needs you!', {
						body: `Job ${ interaction.job_id } requires user input`,
						requireInteraction: true
					} );
					this.rememberNotified( interaction.id );
				}
			},
			closeNotification() {
				if ( this.userNotification ) {
					this.userNotification.close();
					this.userNotification = null;
				}
			},
			reset() {
				this.$reset();
				this.backportJob = null;
				this.alreadyNotifiedInters = '[]';
				this.userNotification = null;
			}
		}
	}
);
