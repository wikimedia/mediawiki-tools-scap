<template>
	<cdx-card
		class="job-card"
		:class="rootClasses"
		:url="jobLink"
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
					{{ user }}
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
					<span v-if="isRunning">
						{{ status }}
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
			</div>
		</template>
		<template #supporting-text>
			<sp-interaction
				v-if="showInteraction && interaction"
				:interaction="interaction"
			/>
			<div
				v-if="!showInteraction && showJobDetails"
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
								<pre
									class="job-card__details__change-info__commit-msg"
									v-html="info.formattedCommitMsg" />
							</div>
						</cdx-accordion>
					</div>
				</div>
				<hr>
				<div class="job-card__details__log">
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
import { CdxCard, CdxInfoChip, CdxAccordion, CdxProgressBar } from '@wikimedia/codex';
import { cdxIconInfoFilled, cdxIconLinkExternal } from '@wikimedia/codex-icons';
import Interaction from '../types/Interaction';
import SpInteraction from './Interaction.vue';
import SpJobLog from './JobLog.vue';
import { useRoute, useRouter } from 'vue-router';
import '@xterm/xterm/css/xterm.css';

export default defineComponent( {
	name: 'SpJobCard',

	components: {
		CdxAccordion,
		CdxCard,
		CdxInfoChip,
		CdxProgressBar,
		SpInteraction,
		SpJobLog
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
			type: String,
			required: false,
			default: null
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
		}
	},

	setup( props ) {
		const route = useRoute();
		const router = useRouter();

		const isLoading = ref( true );
		const error = ref( null );

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
					res += `<a href="${ elt.href }" target="_blank">${ elt.text }</a>`;
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

		// Prevent displaying interaction within a JobCard on JobViewerPage.
		const showInteraction = computed( () => route.name !== 'job' );
		const showJobDetails = computed( () => route.name === 'job' );
		const isRunning = computed( () => props.started_at && !props.finished_at );
		const rootClasses = computed( () => ( {
			'job-card--highlighted': props.started_at && !props.finished_at && !showJobDetails.value,
			'job-card--has-details': showJobDetails.value
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

		const statusChipMessage = computed( () => {
			if ( props.exit_status === null ) {
				return 'Pending';
			} else if ( props.exit_status === 0 ) {
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
			statusType,
			statusChipMessage,
			cdxIconInfoFilled,
			cdxIconLinkExternal,
			rootClasses,
			isRunning,
			showInteraction,
			showJobDetails,
			finishedInfo,
			changeInfos,
			handleClick,
			isLoading,
			error,
			jobLink
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

	&__chip {
		width: fit-content;
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
