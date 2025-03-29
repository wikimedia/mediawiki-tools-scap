<template>
	<sp-backport :idle="idle" />
	<sp-job-history />
</template>

<script lang="ts">
import { defineComponent, ref, onMounted, onUnmounted } from 'vue';
import useApi from '../api';
import SpBackport from './Backport.vue';
import SpJobHistory from './JobHistory.vue';

export default defineComponent( {
	name: 'OverviewPage',
	components: {
		SpBackport,
		SpJobHistory
	},

	setup() {
		const api = useApi();

		// Reactive data properties.
		const idle = ref( false );
		const interaction = ref( null );
		const jobrunnerStatus = ref( 'Jobrunner Status: Unknown' );

		// Non-reactive data.
		const INTERVAL = 1000;
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
				// eslint-disable-next-line no-console
				console.error( `updateJobrunnerStatus caught: ${ error.message }` );
			}
		}

		// Lifecycle hooks.
		onMounted( () => {
			intervalTimer = setInterval( updateJobrunnerStatus, INTERVAL );
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
