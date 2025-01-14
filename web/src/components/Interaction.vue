<template>
	<div class="interaction">
		<div class="interaction__prompt">
			{{ interaction.prompt }}
		</div>
		<div class="interaction__action">
			<cdx-button
				v-for="( code, choice ) in interaction.choices"
				:key="choice"
				:value="code"
				@click="choiceSelected( code )"
			>
				{{ choice }}
			</cdx-button>
		</div>
	</div>
</template>

<script lang="ts">
import { defineComponent, PropType } from 'vue';
import useApi from '../api';
import Interaction from '../types/Interaction';
import { CdxButton } from '@wikimedia/codex';

export default defineComponent( {
	name: 'SpInteraction',

	components: {
		CdxButton
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

		function choiceSelected( code: string ) {
			api.respondInteraction(
				props.interaction.job_id,
				props.interaction.id,
				code
			);
		}

		return {
			choiceSelected
		};
	}
} );
</script>

<style lang="less" scoped>
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

.interaction {
	margin-bottom: @spacing-150;

	&__prompt {
		white-space: pre;
		font-size: @font-size-medium;
		overflow-x: auto;
		padding-bottom: @spacing-50;
	}

	&__action {
		margin-top: @spacing-100;

		button:first-child {
			margin-right: @spacing-25;
		}
	}
}

</style>
