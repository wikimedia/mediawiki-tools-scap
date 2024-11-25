<template>
	<div>
		<h2 v-if="job">
			Job #{{ job.id }}
		</h2>
		<br>
		<sp-interaction
			v-if="interaction"
			:interaction="interaction"
		/>

		<sp-job-card
			v-if="job"
			:key="job.id"
			v-bind="job"
		/>
	</div>
</template>

<script lang="ts">
import { defineComponent, ref, onMounted, onUnmounted } from 'vue';
import useApi from '../api';
import SpInteraction from './Interaction.vue';
import SpJobCard from './JobCard.vue';

export default defineComponent( {
	name: 'JobViewerPage',

	components: {
		SpInteraction,
		SpJobCard
	},

	props: {
		jobId: {
			type: String,
			required: true,
			default: null
		}
	},

	setup( props ) {
		// Pinia store.
		const api = useApi();

		// Reactive data properties.
		const job = ref( null );
		const jobRunning = ref( false );
		const monitorInterval = ref( null );
		const interaction = ref( null );

		// Methods.
		const stopMonitor = () => {
			if ( monitorInterval.value ) {
				clearInterval( monitorInterval.value );
				monitorInterval.value = null;
			}
		};

		const monitorJob = async () => {
			const jobInfo = await api.getJobInfo( props.jobId );
			const fetchedJob = jobInfo.job;

			// eslint-disable-next-line camelcase
			fetchedJob.command_decoded = JSON.parse( fetchedJob.command ).join( ' ' );
			job.value = fetchedJob;
			jobRunning.value = ( fetchedJob.started_at && !fetchedJob.finished_at );
			interaction.value = jobInfo.pending_interaction;

			if ( !jobRunning.value ) {
				stopMonitor();
			}
		};

		// Lifecycle hooks.
		onMounted( () => {
			monitorInterval.value = setInterval( monitorJob, 1000 );
			monitorJob();
		} );

		onUnmounted( () => {
			stopMonitor();
		} );

		return {
			job,
			interaction
		};
	}
} );
</script>
