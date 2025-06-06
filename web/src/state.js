import { defineStore } from 'pinia';
import { useLocalStorage } from '@vueuse/core';

export const notificationsStore = defineStore( 'spiderpig-notifications',
	{
		state() {
			return {
				backportJob: useLocalStorage( 'spiderpig-bp-job', null ),
				alreadyNotifiedInters: useLocalStorage( 'spiderpig-notified-interactions', '[]' )
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
			reset() {
				this.$reset();
				this.backportJob = null;
				this.alreadyNotifiedInters = '[]';
			}
		}
	}
);
