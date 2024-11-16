<template>
	<cdx-card
		class="job-card"
		:class="rootClasses"
	>
		<template #description>
			<div class="job-card__column">
				<div class="job-card__label">
					Id
				</div>
				<router-link :to="{ name: 'job', params: { jobId: job.id } }">
					{{ job.id }}
				</router-link>
			</div>

			<div class="job-card__column">
				<div class="job-card__label">
					Command
				</div>
				{{ getFormattedText( job.command_decoded ) }}
			</div>

			<div class="job-card__column">
				<div class="job-card__label">
					User
				</div>
				{{ job.user }}
			</div>

			<div class="job-card__column">
				<div class="job-card__label">
					Started
				</div>
				{{ getFormattedDate( job.started_at ) }}
			</div>

			<div class="job-card__column">
				<div class="job-card__label">
					Finished
				</div>
				{{ getFormattedDate( job.finished_at ) }}
			</div>

			<div class="job-card__column">
				<div class="job-card__label">
					Status
				</div>
				<cdx-info-chip
					:status="statusType"
					:icon="cdxIconInfoFilled"
					class="job-card__chip"
				>
					{{ getExitStatusMessage( job.exit_status ) }}
				</cdx-info-chip>
			</div>
		</template>
	</cdx-card>
</template>

<script lang="ts">
import { defineComponent, PropType, computed } from 'vue';
import { CdxCard, CdxInfoChip } from '@wikimedia/codex';
import { cdxIconInfoFilled } from '@wikimedia/codex-icons';

const JOB_RUNNING = '..Running...';

interface Job {
	id: number,
	command_decoded: string,
	user: string,
	queued_at?: string,
	started_at?: string,
	finished_at?: string,
	exit_status: number,
	status?: string,
	finished_at_message?: string
}

export default defineComponent( {
	name: 'SpJobCard',

	components: {
		CdxCard,
		CdxInfoChip
	},

	props: {
		job: {
			type: Object as PropType<Job>,
			default: null
		}
	},

	setup( props ) {
		// eslint-disable-next-line arrow-body-style
		const rootClasses = computed( () => {
			return {
				'job-card--highlighted': props.job.exit_status === null
			};
		} );

		const statusType = computed( () => {
			const exit = props.job?.exit_status;

			if ( exit === 0 ) {
				return 'success'
			}

			return 'error';
		} );

		function getFormattedDate( dateString ) {
			// Handle the job-in-progress message by bailing early
			if ( dateString === JOB_RUNNING ) {
				return;
			}

			const date = new Date( dateString );
			return date.toUTCString();
		}

		function getFormattedText( text ) {
			return text.split( ' ' )
				.map( ( word ) => word[ 0 ].toUpperCase() + word.slice( 1 ) )
				.join( ' ' );
		}

		function getExitStatusMessage( exit ) {
			if ( exit === 0 ) {
				return 'Finished';
			}

			return 'Pending';
		}

		return {
			getFormattedDate,
			getFormattedText,
			statusType,
			getExitStatusMessage,
			cdxIconInfoFilled,
			rootClasses
		};
	}
} );
</script>

<style lang="less">
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

.job-card {
	.cdx-card__text__description {
		// Unset CdxCard styles and use grid layout.
		@media screen and ( min-width: @min-width-breakpoint-tablet ) {
			display: grid;
			grid-template-columns: 5% 15% 10% 25% 25% 15%;
			grid-template-rows: auto auto;
			gap: 8px;
			grid-gap: 8px;
			min-width: 680px;
			box-sizing: border-box;
			overflow-x: auto;
			position: unset;
		}
	}

	&--highlighted.cdx-card {
		background-color: @background-color-warning-subtle;
	}

	&__label {
		color: @color-base;
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
