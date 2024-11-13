<template>
	<div class="interaction">
		<cdx-message
			class="interaction__message"
			type="warning"
			allow-user-dismiss
		>
			<router-link :to="{ name: 'job', params: { jobId: interaction.job_id } }">
				Job #{{ interaction.job_id }}
			</router-link>
			is awaiting interaction.
		</cdx-message>
		<div v-if="isJobDetailsPage" class="interaction__prompt">
			{{ interaction.prompt }}
		</div>
		<div v-if="isJobDetailsPage" class="interaction__action">
			<cdx-button
				v-for="( code, choice ) in interaction.choices"
				:key="choice"
				:value="code"
				action="progressive"
				:weight="code === interaction.default ? 'primary' : 'normal'"
				@click="choiceSelected( code )"
			>
				{{ choice }}
			</cdx-button>
		</div>
	</div>
</template>

<script lang="ts">
import { defineComponent, PropType, computed } from 'vue';
import useApi from '../api';
import Interaction from '../types/Interaction';
import { CdxButton, CdxMessage } from '@wikimedia/codex';
import { useRoute } from 'vue-router';

export default defineComponent( {
	name: 'SpInteraction',

	components: {
		CdxButton,
		CdxMessage
	},

	props: {
		interaction: {
			type: Object as PropType<Interaction>,
			required: true
		}
	},

	setup( props ) {
		// Pinia store and router.
		const api = useApi();
		const route = useRoute();

		const isJobDetailsPage = computed( () => route.path.includes( '/jobs' ) );

		function choiceSelected( code: string ) {
			api.respondInteraction(
				props.interaction.job_id,
				props.interaction.id,
				code
			);
		}

		return {
			choiceSelected,
			isJobDetailsPage
		};
	}
} );
</script>

<style lang="less" scoped>
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

.interaction {
	margin-bottom: @spacing-150;
}

.interaction__prompt {
	white-space: pre;
	font-family: 'Courier New', Courier, monospace;
	font-size: 15px;
}
</style>
