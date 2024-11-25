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
					<router-link :to="{ name: 'job', params: { jobId: id } }">
						{{ id }}
					</router-link>
				</div>

				<div class="job-card__column">
					<div class="job-card__label">
						Command
					</div>
					{{ getFormattedText( command_decoded ) }}
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
			<sp-interaction
				v-if="showInteraction && interaction"
				:interaction="interaction"
			/>
			<div
				v-if="!showInteraction && showJobDetails"
				class="job-card__details"
			>
				<hr>
				<div class="job-card__details-info">
					<div class="job-card__details-info__column1">
						<div>
							Queued: {{ getFormattedDate( queued_at ) }}
						</div>
						<div>
							Duration: {{ duration }}
						</div>
						<div>
							Branch:
						</div>
						<div>
							Phab:
						</div>
						<div>
							Topic:
						</div>
						<div>
							Change number:
						</div>
					</div>
					<div class="job-card__details-info__column2">
						<div>
							Change info:
						</div>
					</div>
				</div>
				<hr>
				<div class="job-card__details-log">
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
import { defineComponent, computed, PropType } from 'vue';
import { CdxCard, CdxInfoChip } from '@wikimedia/codex';
import { cdxIconInfoFilled } from '@wikimedia/codex-icons';
import Interaction from '../types/Interaction';
import SpInteraction from './Interaction.vue';
import SpJobLog from './JobLog.vue';
import { useRoute } from 'vue-router';
import '@xterm/xterm/css/xterm.css';

const JOB_RUNNING = '..Running...';

export default defineComponent( {
	name: 'SpJobCard',

	components: {
		CdxCard,
		CdxInfoChip,
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

		// Prevent displaying interaction within a JobCard on JobViewerPage.
		const showInteraction = computed( () => route.name !== 'job' );
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

		function getFormattedText( text:string ) {
			return text.split( ' ' )
				.map( ( word ) => word[ 0 ].toUpperCase() + word.slice( 1 ) )
				.join( ' ' );
		}

		// FIXME: Ensure `duration` ref updates automatically when the job finishes w/o refresh.
		const duration = computed( () => calculateDuration() );

		function calculateDuration() {
			// Create Date objects from the UTC ISO 8601 timestamps.
			const start = new Date( props.started_at );
			const end = props.finished_at ? new Date( props.finished_at ) : null;

			// Indicate that the job is "in progress" when the job is not complete.
			if ( !end ) {
				return 'In progress';
			}

			// Check if the Date objects are valid.
			if ( isNaN( start.getTime() ) || isNaN( end.getTime() ) ) {
				return;
			}

			// Calculate the difference in milliseconds.
			const durationMs = end.getTime() - start.getTime();

			// Convert to more readable formats.
			const durationSeconds = Math.floor( durationMs / 1000 );
			const minutes = Math.floor( durationSeconds / 60 );
			const seconds = durationSeconds % 60;

			// Pad numbers with leading zeros.
			const formattedMinutes = minutes.toString().padStart( 2, '0' );
			const formattedSeconds = seconds.toString().padStart( 2, '0' );

			// Format the duration.
			return `${ formattedMinutes }m ${ formattedSeconds }s`;
		}

		return {
			getFormattedDate,
			getFormattedText,
			statusType,
			statusChipMessage,
			cdxIconInfoFilled,
			rootClasses,
			isRunning,
			showInteraction,
			showJobDetails,
			duration
		};
	}
} );
</script>

<style lang="less">
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

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
			display: grid;
			grid-template-columns: 5% 15% 10% 20% 20% 25%;
			grid-template-rows: auto auto;
			gap: 8px;
			grid-gap: 8px;
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
}
</style>
