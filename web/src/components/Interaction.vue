<template>
	<div class="interaction">
		<div
			class="interaction__prompt"
			v-html="renderedPromptHtml" />
		<div class="interaction__action">
			<cdx-button
				v-for="( code, choice ) in interaction.choices"
				:key="choice"
				:value="code"
				@click.stop.prevent="choiceSelected( code )"
			>
				{{ choice }}
			</cdx-button>
		</div>
	</div>
</template>

<script lang="ts">
import { computed, defineComponent, onMounted, PropType } from 'vue';
import { useRoute, useRouter } from 'vue-router';
import useApi from '../api';
import Interaction from '../types/Interaction';
import { CdxButton } from '@wikimedia/codex';
import { notificationsStore } from '@/state';
import linkifyStr from 'linkify-string';

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
		// Pinia store
		const api = useApi();
		const route = useRoute();
		const router = useRouter();
		const notifications = notificationsStore();
		const linkifyOptions = {
			target: '_blank',
			className: 'interaction__prompt__link',
			defaultProtocol: 'https'
		};

		function choiceSelected( code: string ) {
			api.respondInteraction(
				props.interaction.job_id,
				props.interaction.id,
				code
			);
			notifications.closeNotification();
		}

		// Lifecycle hooks
		onMounted( () => {
			notifications.notifyUser( props.interaction );
		} );

		const renderedPromptHtml = computed( () => {
			const prompt = props.interaction.prompt;
			const failedPromptMatch = prompt.match( /^(.*?failed\.)\n\n(.*)$/s );
			const onCurrentJobPage = route.name === 'job' &&
				String( route.params.jobId ) === String( props.interaction.job_id );

			if (
				props.interaction.type !== 'choices' ||
				!failedPromptMatch ||
				onCurrentJobPage
			) {
				return linkifyStr( prompt, linkifyOptions );
			}

			const [ , prefix, suffix ] = failedPromptMatch;
			const jobLogHref = router.resolve( {
				name: 'job',
				params: { jobId: props.interaction.job_id },
				hash: '#log'
			} ).href;
			const jobLogLink =
				`<a href="${ jobLogHref }" ` +
				`class="interaction__prompt__link">job log</a>`;

			return (
				linkifyStr( prefix, linkifyOptions ) +
				`\nView the ${ jobLogLink } for details.\n\n` +
				linkifyStr( suffix, linkifyOptions )
			);
		} );

		return {
			choiceSelected,
			renderedPromptHtml
		};
	}
} );
</script>

<style lang="less" scoped>
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import ( reference ) '@wikimedia/codex/mixins/link.less';

.interaction {
	margin-bottom: @spacing-150;

	&__prompt {
		white-space: pre;
		font-family: 'Courier New', Courier, monospace;
		font-size: 0.9375rem;
		overflow-x: auto;
		padding-bottom: @spacing-50;

		:deep(&__link) {
			.cdx-mixin-link();
		}
	}

	&__action {
		display: flex;
		justify-content: flex-start;
		align-items: baseline;
		gap: @spacing-50;
	}
}

</style>
