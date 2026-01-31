<template>
	<cdx-card
		class="job-card"
		:class="rootClasses"
	>
		<template #description>
			<div class="job-card__content">
				<div class="job-card__column">
					<div class="job-card__label">
						Id
					</div>
					<strong>{{ id }}</strong>
				</div>

				<div class="job-card__column">
					<div class="job-card__label">
						Command
					</div>
					{{ command_decoded }}
				</div>

				<div class="job-card__column">
					<div class="job-card__label">
						User
					</div>
					{{ getFormattedUser( user ) }}
				</div>

				<div class="job-card__column">
					<div class="job-card__label">
						Started
					</div>
					{{ getFormattedDate( started_at ) }}
				</div>

				<div class="job-card__column">
					<div class="job-card__label">
						Finished
					</div>
					{{ finishedInfo }}
				</div>

				<div class="job-card__column">
					<div class="job-card__label">
						Status
					</div>
					<div class="job-card__status-row">
						<span v-if="running" class="job-card__status">
							{{ status.status }}
						</span>
						<cdx-info-chip
							v-else
							:status="statusType"
							:icon="cdxIconInfoFilled"
							class="job-card__chip"
						>
							{{ statusChipMessage }}
						</cdx-info-chip>
					</div>
					<sp-progress-bar
						v-if="running && status.progress"
						:progress="status.progress"
					/>
				</div>

				<div class="job-card__column job-card__column--actions">
					<div class="job-card__label">
						Actions
					</div>
					<div class="job-card__actions-row">
						<button
							v-if="showViewLogButton"
							class="job-card__view-log-button"
							title="View job log"
							@click="navigateToJob"
						>
							<v-icon icon="mdi-console" />
						</button>
						<button
							class="job-card__stop-button"
							:disabled="!running || stopping"
							:title="
								!running ? 'Job is not running' :
								stopping ? 'Stopping job...' : 'Stop job'
							"
							@click="openStopConfirmation"
						>
							<span class="job-card__stop-button__label">STOP</span>
						</button>
						<button
							class="job-card__retry-button"
							:disabled="!canRetry || retrying"
							:title="
								!canRetry ? `Cannot retry job: ${retryDisabledReason}` :
								retrying ? 'Retrying job...' : 'Retry job'
							"
							@click="openRetryConfirmation"
						>
							<cdx-icon
								:icon="cdxIconReload"
								size="small"
							/>
						</button>
						<cdx-dialog
							v-model:open="confirmStopOpen"
							title="Confirm Stop Job"
							close-button-label="Cancel"
							class="job-card__dialog"
						>
							<p>
								Are you sure you want to stop this running job?
							</p>
							<p v-if="command_decoded">
								<strong>{{ command_decoded }}</strong>
							</p>
							<template #footer>
								<cdx-button
									action="destructive"
									:disabled="stopping"
									@click="confirmStop"
								>
									{{ stopping ? 'Stopping...' : 'Stop' }}
								</cdx-button>
								<cdx-button
									@click="confirmStopOpen = false"
								>
									Cancel
								</cdx-button>
							</template>
						</cdx-dialog>
						<cdx-dialog
							v-model:open="confirmRetryOpen"
							title="Confirm Job Retry"
							close-button-label="Cancel"
							class="job-card__dialog"
						>
							<p>
								Are you sure you want to retry this job?
							</p>
							<p v-if="command_decoded">
								<strong>{{ command_decoded }}</strong>
							</p>
							<cdx-message
								v-if="retryError"
								type="error"
								class="job-card__dialog-error"
							>
								{{ retryError }}
							</cdx-message>
							<template #footer>
								<cdx-button
									action="progressive"
									:disabled="retrying"
									@click="confirmRetry"
								>
									{{ retrying ? 'Retrying...' : 'Retry' }}
								</cdx-button>
								<cdx-button
									@click="closeRetryDialog"
								>
									Cancel
								</cdx-button>
							</template>
						</cdx-dialog>
					</div>
				</div>
			</div>
		</template>
		<template #supporting-text>
			<sp-interaction
				v-if="showInteraction && interaction"
				:interaction="interaction"
			/>
			<div
				v-if="!interaction"
				class="job-card__details"
			>
				<hr>
				<div v-if="isLoading" class="job-card__details__loading">
					<cdx-progress-bar aria-label="Indeterminate progress bar" />
				</div>
				<div v-else-if="error" class="job-card__details__error">
					{{ error }}
				</div>
				<div v-else>
					<div v-for="( info, index ) in changeInfos" :key="index">
						<cdx-accordion
							:open="changeInfos.length === 1"
							:action-icon="cdxIconLinkExternal"
							action-always-visible
							action-button-label="Open external link"
							@action-button-click="handleClick( info.url )"
						>
							<template #title>
								{{ info.number }} {{ info.subject }}
							</template>
							<div class="job-card__details__change-info">
								<div class="job-card__details__change-info__grid">
									<div class="job-card__details__change-info__grid__item">
										<div class="job-card__details__change-info__label">
											Change number
										</div>
										<div>
											<a
												class="job-card__details__change-info__link"
												:href="info.url"
												target="_blank"
											>
												{{ info.number }}
											</a>
										</div>
									</div>
									<div class="job-card__details__change-info__grid__item">
										<div class="job-card__details__change-info__label">
											Repo
										</div>
										<div>
											<a
												class="job-card__details__change-info__link"
												:href="info.repoURL"
												target="_blank"
											>
												{{ info.project }}
											</a>
										</div>
									</div>
									<div class="job-card__details__change-info__grid__item">
										<div class="job-card__details__change-info__label">
											Branch
										</div>
										<div>
											<a
												class="job-card__details__change-info__link"
												:href="info.branchURL"
												target="_blank"
											>
												{{ info.branch }}
											</a>
										</div>
									</div>
								</div>
								<div class="job-card__details__change-info__label">
									Commit message
								</div>
								<!-- eslint-disable vue/no-v-html -->
								<pre
									class="job-card__details__change-info__commit-msg"
									v-html="info.formattedCommitMsg" />
								<!-- eslint-enable vue/no-v-html -->
							</div>
						</cdx-accordion>
					</div>
				</div>
				<hr v-if="showJobLog">
				<div v-if="showJobLog" class="job-card__details__log">
					<sp-job-log
						:job-id="id"
						:is-job-in-progress="started_at && !finished_at"
					>
						Log
					</sp-job-log>
				</div>
			</div>
		</template>
	</cdx-card>
</template>

<script lang="ts">
import { defineComponent, ref, computed, PropType, watch } from 'vue';
import { CdxCard, CdxInfoChip, CdxAccordion, CdxProgressBar, CdxIcon, CdxDialog, CdxButton, CdxMessage } from '@wikimedia/codex';
import { cdxIconInfoFilled, cdxIconLinkExternal, cdxIconReload } from '@wikimedia/codex-icons';
import { VIcon } from 'vuetify/components/VIcon';
import Interaction from '../types/Interaction';
import JobStatus from '../types/JobStatus';
import SpInteraction from './Interaction.vue';
import SpJobLog from './JobLog.vue';
import SpProgressBar from './ProgressBar.vue';

import { useRoute, useRouter } from 'vue-router';
import useApi from '../api';
import useJobrunner from '../jobrunner';
import '@xterm/xterm/css/xterm.css';

export default defineComponent( {
	name: 'SpJobCard',

	components: {
		CdxAccordion,
		CdxCard,
		CdxInfoChip,
		CdxProgressBar,
		CdxIcon,
		CdxDialog,
		CdxButton,
		CdxMessage,
		VIcon,
		SpInteraction,
		SpJobLog,
		SpProgressBar
	},

	props: {
		id: {
			type: Number,
			default: null
		},
		// eslint-disable-next-line camelcase, vue/prop-name-casing
		command_decoded: {
			type: String,
			required: false,
			default: null
		},
		user: {
			type: String,
			required: false,
			default: null
		},
		// eslint-disable-next-line camelcase, vue/prop-name-casing
		started_at: {
			type: Number,
			required: false,
			default: null
		},
		// eslint-disable-next-line camelcase, vue/prop-name-casing
		finished_at: {
			type: Number,
			required: false,
			default: null
		},
		// eslint-disable-next-line camelcase, vue/prop-name-casing
		exit_status: {
			type: Number,
			required: false,
			default: null
		},
		status: {
			type: Object as PropType<JobStatus>,
			required: true
		},
		interaction: {
			type: Object as PropType<Interaction>,
			required: false,
			default: null
		},
		data: {
			type: Object,
			required: false,
			default: null
		},
		duration: {
			type: Number,
			required: false,
			default: null
		},
		running: {
			type: Boolean,
			required: true
		},
		orphaned: {
			type: Boolean,
			required: true
		}
	},

	setup( props ) {
		const route = useRoute();
		const router = useRouter();
		const api = useApi();
		const jobrunner = useJobrunner();

		const isLoading = ref( true );
		const error = ref( null );
		const retrying = ref( false );
		const confirmRetryOpen = ref( false );
		const retryError = ref( null );
		const stopping = ref( false );
		const confirmStopOpen = ref( false );

		// Return a string URL ( if we are not already on the page for this job)
		// or null (if we are already on that page)
		const jobLink = computed( () => {
			const current = router.currentRoute;
			const link = router.resolve( { name: 'job', params: { jobId: props.id } } );
			return link.href !== current.value.path ? link.href : null;
		} );

		const changeInfos = computed( () => {
			const { change_infos: infos } = props.data;
			if ( !infos || infos.length === 0 ) {
				return [];
			}

			return infos.map( formatChangeInfo );
		} );

		function formatCommitMsg( linkifiedCommitMsg ) {
			// Returns an HTML string.
			//
			// NOTE: linkifiedCommitMsg is already processed on the
			// server side to escape potential HTML tags.
			let res = '';

			for ( const elt of linkifiedCommitMsg ) {
				if ( typeof ( elt ) === 'string' ) {
					res += elt;
				} else {
					res += `<a href="${ elt.href }" target="_blank" class="job-card__details__change-info__link">${ elt.text }</a>`;
				}
			}

			return res;
		}

		function formatChangeInfo( changeInfo ) {
			const { linkifiedCommitMsg, subject, project, branch,
				number, url, repoQueryUrl, branchQueryUrl } = changeInfo;
			const formattedCommitMsg = formatCommitMsg( linkifiedCommitMsg );

			return {
				project,
				branch,
				number,
				url,
				repoURL: repoQueryUrl,
				branchURL: branchQueryUrl,
				subject,
				formattedCommitMsg
			};
		}

		const isJobDetailPage = computed( () => route.name === 'job' );
		const showInteraction = computed( () => !isJobDetailPage.value );
		const showJobLog = computed( () => isJobDetailPage.value );
		const showColumnLabels = computed( () => isJobDetailPage.value );
		const rootClasses = computed( () => ( {
			'job-card--highlighted': props.running && !isJobDetailPage.value,
			'job-card--has-details': showColumnLabels.value
		} ) );

		const statusType = computed( () => {
			if ( props.exit_status === 0 ) {
				return 'success';
			} else if ( props.exit_status === 1 ) {
				return 'error';
			} else if ( props.exit_status === null && props.interaction ) {
				return 'warning';
			} else {
				return 'notice';
			}
		} );

		// This computes the content of the status chip of non-running jobs.
		const statusChipMessage = computed( () => {
			if ( !props.started_at ) {
				// Job has not started yet.
				return 'Pending';
			}
			// Everything below here is about a job that has stopped running.

			if ( props.orphaned ) {
				return 'Orphaned';
			}

			if ( props.exit_status === null ) {
				return 'Unknown';
			}

			if ( props.exit_status === 0 ) {
				return 'Finished';
			} else {
				return 'Error';
			}
		} );

		function getFormattedDate( timestamp: number ) {
			if ( !timestamp ) {
				return '';
			}

			const date = new Date( timestamp * 1000 );
			return date.toUTCString();
		}

		function getFormattedUser( user: string ) {
			return {
				// U+00AD SOFT HYPHEN encourages better line wrapping
				'lucaswerkmeister-wmde': 'lucas\u00ADwerkmeister-wmde'
				// (more user names can be added here as needed)
			}[ user ] ?? user;
		}

		function prettyDuration( durationSeconds: number ) {
			durationSeconds = Math.floor( durationSeconds );
			// Convert to more readable format
			const minutes = Math.floor( durationSeconds / 60 );
			const seconds = durationSeconds % 60;

			// Pad numbers with leading zeros.
			const formattedMinutes = minutes.toString().padStart( 2, '0' );
			const formattedSeconds = seconds.toString().padStart( 2, '0' );

			// Format the duration.
			return `${ formattedMinutes }m ${ formattedSeconds }s`;
		}

		const finishedInfo = computed( () => {
			if ( !props.started_at ) {
				return 'Not started yet';
			}

			if ( props.orphaned ) {
				return 'Unknown (orphaned)';
			}

			const duration = prettyDuration( props.duration );

			if ( props.finished_at ) {
				const finishedTimestamp = getFormattedDate( props.finished_at );
				return `${ finishedTimestamp } (Duration ${ duration })`;
			}

			// Job has started but hasn't finished yet
			return `Running for ${ duration }`;

		} );

		function handleClick( url ) {
			// Open a new window to the Gerrit change number url.
			window.open( url );
		}

		function navigateToJob() {
			if ( jobLink.value ) {
				router.push( { name: 'job', params: { jobId: props.id } } );
			}
		}

		const canRetry = computed( () => (
			props.finished_at &&
			!props.running &&
			props.started_at !== null &&
			jobrunner.idle.value
		) );

		const retryDisabledReason = computed( () => {
			if ( !props.finished_at ) {
				return 'Job has not finished yet';
			}
			if ( props.running ) {
				return 'Job is still running';
			}
			if ( props.started_at === null ) {
				return 'Job has not started';
			}
			if ( !jobrunner.idle.value ) {
				return 'Another job is running';
			}
			return '';
		} );

		const showViewLogButton = computed( () => !isJobDetailPage.value );

		function openRetryConfirmation() {
			confirmRetryOpen.value = true;
		}

		function closeRetryDialog() {
			confirmRetryOpen.value = false;
			retryError.value = null;
		}

		async function confirmRetry() {
			try {
				retrying.value = true;
				retryError.value = null;
				const response = await api.retryJob( props.id );
				closeRetryDialog();
				// Navigate to the new job only if we're on the job viewer page
				if ( response.id && isJobDetailPage.value ) {
					router.push( { name: 'job', params: { jobId: response.id } } );
				}
			} catch ( err: unknown ) {
				const message = err instanceof Error ? err.message : 'Failed to retry job.';
				retryError.value = message;
				error.value = message;
			} finally {
				retrying.value = false;
			}
		}

		function openStopConfirmation() {
			confirmStopOpen.value = true;
		}

		async function confirmStop() {
			try {
				stopping.value = true;
				await api.signalJob( props.id, 'interrupt' );
				confirmStopOpen.value = false;
			} catch ( err: unknown ) {
				error.value = err instanceof Error ? err.message : 'Failed to stop job.';
			} finally {
				stopping.value = false;
			}
		}

		watch(
			() => props.data, ( newData ) => {
				if ( !newData ) {
					isLoading.value = false;
					error.value = 'No data received.';
					return;
				}

				isLoading.value = false;
				error.value = null;
			},
			{ immediate: true }
		);

		return {
			getFormattedDate,
			getFormattedUser,
			statusType,
			statusChipMessage,
			cdxIconInfoFilled,
			cdxIconLinkExternal,
			cdxIconReload,
			rootClasses,
			showInteraction,
			showJobLog,
			showViewLogButton,
			finishedInfo,
			changeInfos,
			handleClick,
			navigateToJob,
			isLoading,
			error,
			retryError,
			canRetry,
			retryDisabledReason,
			openRetryConfirmation,
			closeRetryDialog,
			confirmRetry,
			confirmRetryOpen,
			retrying,
			openStopConfirmation,
			confirmStop,
			confirmStopOpen,
			stopping
		};
	}
} );
</script>

<style lang="less">
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import ( reference ) '@wikimedia/codex/mixins/link.less';
@import '../mixins/job-grid.less';

.job-card {
	a {
		text-decoration: none;
	}

	.cdx-card__text {
		width: 100%;

	}

	// Show job card labels on job details page, otherwise hide labels.
	&:not( &--has-details ) &__label {
		@media screen and ( min-width: @min-width-breakpoint-tablet ) {
			display: none;
		}
	}

	&__content {
		// Unset CdxCard styles and apply grid layout on mid to large screen devices.
		@media screen and ( min-width: @min-width-breakpoint-tablet ) {
			.sp-mixin-job-grid();
		}
	}

	&--highlighted.cdx-card {
		background-color: @background-color-warning-subtle;
	}

	&__label {
		font-weight: @font-weight-bold;
	}

	&__column {
		display: flex;
		flex-direction: column;
	}

	&__status {
		font-weight: bold;
	}

	&__status-row {
		display: inline-flex;
		align-items: center;
		gap: @spacing-50;
	}

	&__chip {
		width: fit-content;
	}

	&__column--actions {
		text-align: right;
	}

	&__actions-row {
		display: inline-flex;
		align-items: center;
		justify-content: flex-end;
		gap: @spacing-50;
		flex-direction: row;

		@media screen and ( max-width: 850px ) {
			flex-direction: column;
		}
	}

	&__view-log-button,
	&__retry-button {
		display: inline-flex;
		align-items: center;
		justify-content: center;
		width: 28px;
		height: 28px;
		background-color: @background-color-neutral-subtle;
		border: 1px solid @border-color-interactive;
		cursor: pointer;
		padding: 0;

		&:disabled {
			opacity: 0.5;
		}

		.v-icon,
		.cdx-icon {
			width: 20px;
			height: 20px;
			font-size: 20px;
		}
	}

	&__dialog footer {
		display: flex;
		justify-content: flex-end;
		align-items: baseline;
		gap: @spacing-50;
		padding: @spacing-100;
	}

	&__stop-button {
		display: inline-flex;
		align-items: center;
		justify-content: center;
		width: 28px;
		height: 28px;
		background-color: #d33;
		border: 1px solid #d33;
		clip-path: polygon(30% 0, 70% 0, 100% 30%, 100% 70%, 70% 100%, 30% 100%, 0 70%, 0 30%);
		cursor: pointer;

		&:disabled {
			opacity: 0.5;
		}

		&__label {
			color: #fff;
			font-weight: @font-weight-bold;
			font-size: 10px;
			line-height: 1;
			letter-spacing: 0.5px;
			user-select: none;
		}
	}

	&__details {
		&__loading,
		&__error {
			padding-top: @spacing-50;
			padding-bottom: @spacing-50;
		}

		&__change-info {
			&__grid {
				@media screen and ( min-width: @min-width-breakpoint-tablet ) {
					display: grid;
					grid-template-columns: repeat( 4, 1fr );
					gap: @spacing-12;
					grid-gap: @spacing-12;
					padding-bottom: @spacing-100;
				}
			}

			&__label {
				font-weight: @font-weight-semi-bold;
				padding-top: @spacing-50;
			}

			&__link {
				.cdx-mixin-link();
			}

			&__subject {
				padding-top: @spacing-50;
			}

			&__commit-msg {
				font-family: inherit;
				// Resolve `<pre>`tag content overflow.
				overflow: auto;
			}

			&__bug {
				padding-bottom: @spacing-50;
			}
		}

		&__log {
			font-weight: @font-weight-semi-bold;
		}
	}
}
</style>
