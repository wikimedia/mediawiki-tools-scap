<template>
	<div id="jobrunner-status">
		{{ jobrunnerStatus }}
	</div>
	<sp-backport v-if="idle" />
	<sp-interaction
		v-if="interaction"
		:interaction="interaction"
	/>
	<sp-job-history />
</template>

<script lang="ts">
import { defineComponent, ref, onMounted, onUnmounted } from 'vue';
import useApi from '../api';
import { useRouter } from 'vue-router';
import SpBackport from './Backport.vue';
import SpJobHistory from './JobHistory.vue';
import SpInteraction from './Interaction.vue';

export default defineComponent( {
	name: 'OverviewPage',
	components: {
		SpBackport,
		SpJobHistory,
		SpInteraction
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
		async function updateJobrunnerStatus() {
			try {
				const res = await api.getJobrunnerStatus();
				jobrunnerStatus.value = `Jobrunner Status: ${ res.status }`;
				idle.value = res.status === 'idle';
				interaction.value = res.pending_interaction;
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
			idle,
			interaction,
			jobrunnerStatus
		};
	}
} );
</script>

<style scoped>
</style>
