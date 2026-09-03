<template>
	<div class="interaction">
		<div
			class="interaction__prompt"
			@click="onPromptClick"
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
import { computed, defineComponent, PropType, watch } from 'vue';
import { useRoute, useRouter } from 'vue-router';
import useApi from '../api';
import Interaction from '../types/Interaction';
import { CdxButton } from '@wikimedia/codex';
import { notificationsStore } from '@/state';
import linkifyStr from 'linkify-string';

// The prompt that scap deploy-service --confirm-diffs asks.
const DIFF_PROMPT_RE = /^Note: Diffs are relative to/;

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

		// v-html cannot hold a router-link, so the job log link goes through
		// the router here.  The other links that may be in the prompt are
		// handled normally.
		function onPromptClick( event: MouseEvent ) {
			const anchor = ( event.target as HTMLElement ).closest(
				'a[data-job-log]'
			);
			if ( !anchor ) {
				return;
			}

			event.preventDefault();
			router.push( anchor.getAttribute( 'href' ) );
		}

		function choiceSelected( code: string ) {
			api.respondInteraction(
				props.interaction.job_id,
				props.interaction.id,
				code
			);
			notifications.closeNotification();
		}

		// Notify when a new interaction arrives, even if this component instance
		// is reused and does not remount.
		watch(
			() => props.interaction.id,
			() => {
				notifications.notifyUser( props.interaction );
			},
			{ immediate: true }
		);

		const renderedPromptHtml = computed( () => {
			const prompt = props.interaction.prompt;
			const onCurrentJobPage = route.name === 'job' &&
				String( route.params.jobId ) === String( props.interaction.job_id );

			if ( props.interaction.type !== 'choices' || onCurrentJobPage ) {
				return linkifyStr( prompt, linkifyOptions );
			}

			// review_diffs() in scap/kubernetes.py writes the diffs to the job
			// log as sensitive lines, so the link for that prompt needs to include
			// the query parameter that allows viewing sensitive lines.
			const opensSensitive = DIFF_PROMPT_RE.test( prompt );
			const jobLogHref = router.resolve( {
				name: 'job',
				params: { jobId: props.interaction.job_id },
				query: opensSensitive ? { sensitive: '1' } : {},
				hash: '#log'
			} ).href;
			const jobLogLine =
				`View the <a href="${ jobLogHref }" data-job-log ` +
				`class="interaction__prompt__link">job log</a> for details.`;

			// A prompt that reports a failure reads better with the line after
			// the report and before the question.
			const failedPromptMatch = prompt.match( /^(.*?failed\.)\n\n(.*)$/s );
			if ( failedPromptMatch ) {
				const [ , prefix, suffix ] = failedPromptMatch;
				return (
					linkifyStr( prefix, linkifyOptions ) +
					`\n${ jobLogLine }\n\n` +
					linkifyStr( suffix, linkifyOptions )
				);
			}

			return `${ jobLogLine }\n\n` + linkifyStr( prompt, linkifyOptions );
		} );

		return {
			choiceSelected,
			onPromptClick,
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
