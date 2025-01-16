<template>
	<div class="job-view__header">
		<h2 v-if="job">
			Job #{{ job.id }}
		</h2>
		<div class="job-view__header__buttons">
			<cdx-button
				@click="$router.push( '/' )"
			>
				Close
			</cdx-button>
			<cdx-menu-button
				v-show="jobRunning"
				v-model:selected="selection"
				:menu-items="menuItems"
				aria-label="Choose an option"
				@update:selected="onSelect"
			>
				<cdx-icon :icon="cdxIconEllipsis" />
			</cdx-menu-button>
		</div>
	</div>
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
</template>

<script lang="ts">
import { defineComponent, ref, onMounted, onUnmounted } from 'vue';
import useApi from '../api';
import SpInteraction from './Interaction.vue';
import SpJobCard from './JobCard.vue';
import { CdxButton, CdxMenuButton, CdxIcon } from '@wikimedia/codex';
import { cdxIconEllipsis } from '@wikimedia/codex-icons';

export default defineComponent( {
	name: 'JobViewerPage',

	components: {
		SpInteraction,
		SpJobCard,
		CdxButton,
		CdxMenuButton,
		CdxIcon
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
			{ label: 'Interrupt Job', value: 'interrupt' },
			{ label: 'Kill Job (not recommended)', value: 'kill', action: 'destructive' }
		];

		// Reactive data properties.
		const job = ref( null );
		const jobRunning = ref( false );
		const monitorInterval = ref( null );
		const interaction = ref( null );
		const selection = ref( null );

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

			// Reset the selection of menu buttons.
			selection.value = null;
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
			selection,
			menuItems,
			onSelect
		};
	}
} );
</script>

<style lang="less">
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

.job-view {
	&__header {
		display: flex;
		justify-content: space-between;

		&__buttons {
				display: flex;
				gap: @spacing-75;
		}
	}
}
</style>
