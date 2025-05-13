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
				console.log( `setUserNotifications called with jobId: ${ jobId } of type ${ typeof ( jobId ) }` );
				this.reset();
				if ( Notification.permission === 'default' ) {
					await Notification.requestPermission();
				}
				if ( Notification.permission === 'granted' ) {
					this.backportJob = jobId;
				}
			},
			userShouldBeNotified( interaction ) {
				console.log( `userShouldBeNotified: Notification.permission is ${ Notification.permission }, interaction.job_id is ${ interaction.job_id }, of type ${ typeof ( interaction.job_id ) }` );
				return Notification.permission === 'granted' &&
				interaction.job_id === this.backportJob &&
				!this.alreadyNotified( interaction.id );
			},
			alreadyNotified( interactionId ) {
				console.log( ` alreadyNotified: interactionId is ${ interactionId }, of type ${ typeof ( interactionId ) }` );
				return JSON.parse( this.alreadyNotifiedInters ).includes( interactionId );
			},
			rememberNotified( interactionId ) {
				console.log( ` rememberNotified: interactionId is ${ interactionId }, of type ${ typeof ( interactionId ) }` );
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
