<template>
	<div class="interaction">
		<h2>Job {{ interaction.job_id }} is awaiting interaction</h2>
		<div class="prompt">
			{{ interaction.prompt }}
		</div>
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

<style scoped>
.interaction {
	display: table;
	border-style: solid;
	border-width: 1px;
	border-radius: 10px;
	padding: 5px;
	margin: 5px;
}

.prompt {
	white-space: pre;
	font-family: 'Courier New', Courier, monospace;
	font-size: 15px;
}
</style>
