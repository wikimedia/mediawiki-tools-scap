<template>
	<div class="job-view">
		<sp-interaction
			v-if="interaction"
			:interaction="interaction"
		/>

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
					v-show="pending"
					v-model:selected="selection"
					:menu-items="menuItems"
					aria-label="Choose an option"
					@update:selected="onSelect"
				>
					<cdx-icon :icon="cdxIconEllipsis" />
				</cdx-menu-button>
			</div>
		</div>

		<sp-job-card
			v-if="job"
			:key="job.id"
			v-bind="job"
		>
			<div class="job-details">
				<div v-if="pending" class="job-details__loading">
					<cdx-progress-bar aria-label="Indeterminate progress bar" />
				</div>
				<div v-else-if="error" class="job-details__error">
					{{ error }}
				</div>
				<div v-else>
					<hr>

					<div v-for="( change, index ) in changes" :key="index">
						<cdx-accordion
							:open="changes.length === 1"
							:action-icon="cdxIconLinkExternal"
							action-always-visible
							action-button-label="Open external link"
							@action-button-click="onActionButtonClick( change.url )"
						>
							<template #title>
								{{ change.number }} {{ change.subject }}
							</template>

							<sp-change-details v-bind="change" />
						</cdx-accordion>
					</div>
				</div>
			</div>
			<hr>
			<sp-job-log
				:job-id="job.id"
				:is-job-in-progress="job.started_at && !job.finished_at"
			>
				Log
			</sp-job-log>
		</sp-job-card>
	</div>
</template>

<script lang="ts">
import { defineComponent, ref, onMounted, onUnmounted, computed } from 'vue';
import { CdxButton, CdxMenuButton, CdxIcon, CdxProgressBar, CdxAccordion, MenuButtonItemData } from '@wikimedia/codex';
import { cdxIconEllipsis, cdxIconLinkExternal } from '@wikimedia/codex-icons';
import SpChangeDetails from './ChangeDetails.vue';
import SpInteraction from './Interaction.vue';
import SpJobCard from './JobCard.vue';
import SpJobLog from './JobLog.vue';
import useApi from '../api';
import Interaction from '../types/Interaction';

// @todo move this to some kind of utility file and import it here
function formatChangeData( changeData ) {
	const { commit_msg: commitMsg, project, branch, number, url } = changeData;
	const splitCommitMsg = commitMsg?.split( '\n' ) || [];
	const bugs = splitCommitMsg
		.filter( ( line ) => line.startsWith( 'Bug:' ) )
		.map( ( line ) => {
			const bug = line.split( ' ' )[ 1 ];
			return {
				id: bug,
				url: `https://phabricator.wikimedia.org/${ bug }`
			};
		} );

	const formattedCommitMsg = splitCommitMsg
		.slice( 1 )
		.filter( ( line ) => !line.startsWith( 'Bug:' ) && !line.startsWith( 'Change-Id:' ) )
		.join( '\n' )
		.trimEnd();

	const subject = splitCommitMsg[ 0 ] || null;
	const changeId = splitCommitMsg.find( ( line ) => line.startsWith( 'Change-Id:' ) )?.split( ' ' )[ 1 ] || null;

	return {
		project,
		branch,
		number,
		url,
		repoURL: `https://gerrit.wikimedia.org/r/q/project:${ project }`,
		branchURL: branch ? `https://gerrit.wikimedia.org/r/q/branch:${ branch }` : null,
		changeIdURL: changeId ? `https://gerrit.wikimedia.org/r/q/${ changeId }` : null,
		subject,
		formattedCommitMsg,
		bugs,
		changeId
	};
}

export default defineComponent( {
	name: 'JobViewerPage',

	components: {
		CdxAccordion,
		CdxButton,
		CdxIcon,
		CdxMenuButton,
		CdxProgressBar,
		SpChangeDetails,
		SpInteraction,
		SpJobCard,
		SpJobLog
	},

	props: {
		jobId: {
			type: String,
			required: true,
			default: null
		}
	},

	setup( props ) {
		const api = useApi();
		let pollingInterval: ReturnType<typeof setInterval> = null;

		// Non-reactive data.
		const menuItems: MenuButtonItemData[] = [
			{ label: 'Interrupt Job', value: 'interrupt' },
			{ label: 'Kill Job (not recommended)', value: 'kill', action: 'destructive' }
		];

		// Reactive data properties.
		const job = ref( null );
		const pending = ref( true );
		const error = ref( null );
		const selection = ref( null );

		/**
		 * Treat jobs as in progress if there is a "started_at" value but no
		 * "finished_at" value.
		 */
		const isRunning = computed(
			() => job.value ? job.value.started_at && !job.value.finished_at : false
		);

		/**
		 * Any pending Interaction currently associated with the latest fetched job
		 */
		const interaction = computed<Interaction>(
			() => job.value ? job.value.pending_interaction : null
		);

		/**
		 * Array of one or more changes associated with the job.
		 * Each change item includes various properties from the
		 * git commit (repo, subject, bugs, etc).
		 */
		const changes = computed( () => {
			if ( !job.value ) {
				return [];
			}

			const { change_infos: changeData } = JSON.parse( job.value.data );
			return changeData ? changeData.map( formatChangeData ) : [];
		} );

		/**
		 * Fetch the Job data from the API
		 */
		async function fetchJobDetails() {
			const jobInfo = await api.getJobInfo( props.jobId );
			const fetchedJob = jobInfo.job;

			// eslint-disable-next-line camelcase
			fetchedJob.command_decoded = JSON.parse( fetchedJob.command ).join( ' ' );
			job.value = fetchedJob;
			pending.value = false;

			if ( !isRunning.value ) {
				stopMonitor();
			}
		}

		/**
		 * Stop polling the API for Job data.
		 */
		function stopMonitor() {
			if ( pollingInterval ) {
				clearInterval( pollingInterval );
				pollingInterval = null;
			}
		}

		function clickSignalButton( action: string ) {
			api.signalJob( props.jobId, action );
		}

		function onSelect( newSelection: string ) {
			// Handle menu button events.
			clickSignalButton( newSelection );

			// Reset the selection of menu buttons.
			selection.value = null;
		}

		/**
		 * Open the page for the Gerrit patch in a new tab.
		 *
		 * @param {string} url Change URL
		 */
		function onActionButtonClick( url: string ) {
			window.open( url );
		}

		// Lifecycle hooks.
		onMounted( () => {
			pollingInterval = setInterval( fetchJobDetails, 1000 );
			fetchJobDetails();
		} );

		onUnmounted( () => {
			stopMonitor();
		} );

		return {
			job,
			interaction,
			selection,
			menuItems,
			onSelect,
			onActionButtonClick,
			changes,
			pending,
			error,
			cdxIconLinkExternal,
			cdxIconEllipsis
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
		margin-bottom: @spacing-75;

		&__buttons {
				display: flex;
				gap: @spacing-75;
		}
	}
}
</style>
