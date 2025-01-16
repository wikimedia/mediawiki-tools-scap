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
						Queued
					</div>
					{{ getFormattedDate( queued_at ) }}
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
					{{ getFormattedDate( finished_at ) }}
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
			<!-- Job viewer page will pass in additional details here-->
			<slot />
		</template>
	</cdx-card>
</template>

<script lang="ts">
import { defineComponent, computed, PropType } from 'vue';
import { CdxCard, CdxInfoChip } from '@wikimedia/codex';
import { cdxIconInfoFilled } from '@wikimedia/codex-icons';
import Interaction from '../types/Interaction';
import { useRoute, useRouter } from 'vue-router';
import '@xterm/xterm/css/xterm.css';

const JOB_RUNNING = '..Running...';

export default defineComponent( {
	name: 'SpJobCard',

	components: {
		CdxCard,
		CdxInfoChip
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
			type: String,
			required: false,
			default: null
		},
		// eslint-disable-next-line camelcase, vue/prop-name-casing
		finished_at: {
			type: String,
			required: false,
			default: null
		},
		// eslint-disable-next-line camelcase, vue/prop-name-casing
		queued_at: {
			type: String,
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
		}
	},

	setup( props ) {
		const route = useRoute();
		const router = useRouter();

		// Return a string URL ( if we are not already on the page for this job)
		// or null (if we are already on that page)
		const jobLink = computed( () => {
			const current = router.currentRoute;
			const link = router.resolve( { name: 'job', params: { jobId: props.id } } );
			return link.href !== current.value.path ? link.href : null;
		} );

		// Prevent displaying interaction within a JobCard on JobViewerPage.
		const showJobDetails = computed( () => route.name === 'job' );
		const isRunning = computed( () => props.started_at && !props.finished_at );
		const rootClasses = computed( () => ( {
			'job-card--highlighted': props.exit_status !== 0 && !showJobDetails.value,
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

		function getFormattedDate( dateString ) {
			// Handle the job-in-progress message.
			if ( !dateString || dateString === JOB_RUNNING ) {
				return '';
			}

			const date = new Date( dateString );
			return date.toUTCString();
		}

		// @TODO restore the calculatedDuration computed property
		// once we know where we want to display this data.

		return {
			getFormattedDate,
			statusType,
			statusChipMessage,
			cdxIconInfoFilled,
			rootClasses,
			isRunning,
			jobLink
		};
	}
} );
</script>

<style lang="less">
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import ( reference ) '@wikimedia/codex/mixins/link.less';

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
		// Ensure matching grid layout styles for the JobHistory column labels and JobCard content.
		@media screen and ( min-width: @min-width-breakpoint-tablet ) {
			display: grid;
			grid-template-columns: 5% 14% 8% 16% 16% 16% 15%;
			grid-template-rows: auto auto;
			gap: @spacing-50;
			grid-gap: @spacing-50;
			box-sizing: border-box;
			position: unset;
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

		&__log {
			font-weight: @font-weight-semi-bold;
		}
	}
}
</style>
