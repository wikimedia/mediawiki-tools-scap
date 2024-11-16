<template>
	<sp-backport :idle="idle" />
	<sp-job-history />
</template>

<script lang="ts">
import { defineComponent, ref, onMounted, onUnmounted } from 'vue';
import useApi from '../api';
import { useRouter } from 'vue-router';
import SpBackport from './Backport.vue';
import SpJobHistory from './JobHistory.vue';

export default defineComponent( {
	name: 'OverviewPage',
	components: {
		SpBackport,
		SpJobHistory
	},

	setup() {
		// Pinia store and router.
		const api = useApi();
		const router = useRouter();

		// Reactive data properties.
		const idle = ref( false );
		const interaction = ref( null );
		const jobrunnerStatus = ref( 'Jobrunner Status: Unknown' );

		// Non-reactive data.
		let intervalTimer = null;

		// Methods.
		async function updateInteraction( id ) {
			const jobinfo = await api.getJobInfo( id );

			interaction.value = jobinfo.pending_interaction;
		}

		async function updateJobrunnerStatus() {
			try {
				const res = await api.getJobrunnerStatus();
				jobrunnerStatus.value = `Jobrunner Status: ${ res.status }`;
				idle.value = res.status === 'idle';

				if ( res.job_id ) {
					updateInteraction( res.job_id );
				}
			} catch ( error ) {
				if ( !api.isAuthenticated ) {
					router.push( '/login' );
				} else {
					// eslint-disable-next-line no-console
					console.error( `updateJobrunnerStatus caught: ${ error.message }` );
				}
			}
		}

		// Lifecycle hooks.
		onMounted( () => {
			intervalTimer = setInterval( updateJobrunnerStatus, 1000 );
			updateJobrunnerStatus();
		} );

		onUnmounted( () => {
			if ( intervalTimer ) {
				clearInterval( intervalTimer );
				intervalTimer = null;
			}
		} );

		return {
			idle
		};
	}
} );
</script>

<style scoped>
</style>
