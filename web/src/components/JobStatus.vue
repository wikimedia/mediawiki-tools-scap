<template>
	<div class="sp-job-status">
		<div class="sp-job-status__row">
			<span v-if="running" class="sp-job-status__text">
				{{ status.status }}
			</span>
			<cdx-info-chip
				v-else
				:status="statusType"
				:icon="cdxIconInfoFilled"
				class="sp-job-status__chip"
			>
				{{ statusChipMessage }}
			</cdx-info-chip>
		</div>
		<sp-progress-bar
			v-if="running && status.progress"
			:progress="status.progress"
		/>
	</div>
</template>

<script lang="ts">
import { computed, defineComponent, PropType } from 'vue';
import { CdxInfoChip } from '@wikimedia/codex';
import { cdxIconInfoFilled } from '@wikimedia/codex-icons';
import JobStatus from '../types/JobStatus';
import Interaction from '../types/Interaction';
import SpProgressBar from './ProgressBar.vue';

export default defineComponent( {
	name: 'SpJobStatus',

	components: {
		CdxInfoChip,
		SpProgressBar
	},

	props: {
		status: {
			type: Object as PropType<JobStatus>,
			required: true
		},
		running: {
			type: Boolean,
			required: true
		},
		orphaned: {
			type: Boolean,
			default: false
		},
		startedAt: {
			type: Number,
			default: null
		},
		exitStatus: {
			type: Number,
			default: null
		},
		interaction: {
			type: Object as PropType<Interaction>,
			default: null
		}
	},

	setup( props ) {
		const statusType = computed( () => {
			if ( props.exitStatus === 0 ) {
				return 'success';
			} else if ( props.exitStatus === 1 ) {
				return 'error';
			} else if ( props.exitStatus === null && props.interaction ) {
				return 'warning';
			} else {
				return 'notice';
			}
		} );

		// This computes the content of the status chip of non-running jobs.
		const statusChipMessage = computed( () => {
			if ( !props.startedAt ) {
				// Job has not started yet.
				return 'Pending';
			}
			// Everything below here is about a job that has stopped running.

			if ( props.orphaned ) {
				return 'Orphaned';
			}

			if ( props.exitStatus === null ) {
				return 'Unknown';
			}

			if ( props.exitStatus === 0 ) {
				return 'Finished';
			} else {
				return 'Error';
			}
		} );

		return {
			statusType,
			statusChipMessage,
			cdxIconInfoFilled
		};
	}
} );
</script>

<style lang="less" scoped>
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

.sp-job-status {
	&__row {
		display: inline-flex;
		align-items: center;
		gap: @spacing-50;
	}

	&__text {
		font-weight: bold;
		word-break: break-word;
	}

	&__chip {
		width: fit-content;
	}
}
</style>
