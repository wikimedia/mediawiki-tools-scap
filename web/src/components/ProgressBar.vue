<template>
	<v-progress-linear
		v-if="progress"
		v-model="progressPercent"
		color="primary"
		height="22" />
	<div class="progressbar_text">
		{{ progressText }}
	</div>
</template>

<script lang="ts">

import SpiderpigProgressReportRecord from '@/types/SpiderpigProgressReportRecord';
import { computed, defineComponent, PropType } from 'vue';
import { VProgressLinear } from 'vuetify/lib/components/index.mjs';

export default defineComponent( {
	name: 'SpProgressBar',

	components: {
		VProgressLinear
	},

	props: {
		progress: {
			type: Object as PropType<SpiderpigProgressReportRecord>,
			required: true
		}
	},

	setup( props ) {
		const progressPercent = computed( () => {
			const progress = props.progress;

			if ( !progress ) {
				return undefined;
			}

			return progress.tasksPercentComplete;

		} );

		const progressText = computed( () => {
			const progress = props.progress;

			if ( !progress ) {
				return undefined;
			}

			let res = `${ progress.tasksPercentComplete }% (`;

			if ( progress.tasksInFlight !== null ) {
				res += `in-flight: ${ progress.tasksInFlight }; `;
			}
			res += `ok: ${ progress.tasksFinishedOk }; fail: ${ progress.tasksFinishedFailed }; left: ${ progress.tasksRemaining })`;

			return res;

		} );

		return {
			progressPercent,
			progressText
		};
	}
} );

</script>

<style lang="less" scoped>
</style>
