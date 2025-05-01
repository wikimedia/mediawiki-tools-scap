<template>
	<v-toolbar v-if="job">
		<template #title>
			<h2>Job #{{ job.id }}</h2>
		</template>
		<template #append>
			<v-btn
				variant="elevated"
				@click="$router.back()"
			>
				Close
			</v-btn>
			<v-menu>
				<template #activator="{ props }">
					<v-btn
						v-show="jobRunning"
						variant="elevated"
						v-bind="props"
					>
						<cdx-icon :icon="cdxIconEllipsis" />
					</v-btn>
				</template>
				<v-list>
					<v-list-item
						v-for="( item, i ) in menuItems"
						:key="i"
						v-bind="item"
						@click="onSelect( item.value )"
					/>
				</v-list>
			</v-menu>
		</template>
	</v-toolbar>
	<v-sheet color="surface-light">
		<sp-interaction
			v-if="interaction"
			:interaction="interaction"
		/>

		<sp-job-card
			v-if="job"
			:key="job.id"
			v-bind="job"
		/>
	</v-sheet>
</template>

<script lang="ts">
import { defineComponent, ref, onMounted, onUnmounted } from 'vue';
import { VSheet } from 'vuetify/components/VSheet';
import { VToolbar } from 'vuetify/components/VToolbar';
import useApi from '../api';
import SpInteraction from './Interaction.vue';
import SpJobCard from './JobCard.vue';
import { CdxIcon } from '@wikimedia/codex';
import { cdxIconEllipsis } from '@wikimedia/codex-icons';

export default defineComponent( {
	name: 'JobViewerPage',

	components: {
		SpInteraction,
		SpJobCard,
		CdxIcon,
		VSheet,
		VToolbar
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

		// Non-reactive data.
		const menuItems = [
			{ title: 'Interrupt Job', value: 'interrupt' },
			{ title: 'Kill Job (not recommended)', value: 'kill', class: 'bg-error' }
		];

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

		const clickSignalButton = ( action ) => {
			api.signalJob( props.jobId, action );
		};

		const onSelect = ( newSelection ) => {
			// Handle menu button events.
			clickSignalButton( newSelection );
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
			jobRunning,
			interaction,
			cdxIconEllipsis,
			menuItems,
			onSelect
		};
	}
} );
</script>
