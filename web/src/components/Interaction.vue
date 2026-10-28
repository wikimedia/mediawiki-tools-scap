<template>
	<div class="interaction">
		<cdx-accordion
			v-if="hasDetails"
			:key="interaction.id"
			class="interaction__details"
			@toggle="onDetailsToggle"
		>
			<template #title>
				{{ interaction.details_title || 'Details' }}
			</template>
			<cdx-progress-bar
				v-if="detailsLoading"
				inline
				aria-label="Loading the details"
			/>
			<cdx-message v-else-if="detailsError" type="error" inline>
				{{ detailsError }}
			</cdx-message>
			<!-- eslint-disable vue/no-v-html -->
			<pre
				v-else-if="detailsHtml !== null"
				class="interaction__details__text"
				v-html="detailsHtml" />
			<!-- eslint-enable vue/no-v-html -->
		</cdx-accordion>
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
import { computed, defineComponent, PropType, ref, watch } from 'vue';
import { useRoute, useRouter } from 'vue-router';
import useApi from '../api';
import Interaction from '../types/Interaction';
import { CdxAccordion, CdxButton, CdxMessage, CdxProgressBar } from '@wikimedia/codex';
import { notificationsStore } from '@/state';
import linkifyStr from 'linkify-string';
import { AnsiUp } from 'ansi_up';

export default defineComponent( {
	name: 'SpInteraction',

	components: {
		CdxAccordion,
		CdxButton,
		CdxMessage,
		CdxProgressBar
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
			notifications.closeNotification( props.interaction.job_id );
		}

		// The details can be sensitive, so they stay hidden until the user
		// asks for them.
		const hasDetails = computed( () => !!props.interaction.has_details );
		const detailsHtml = ref<string|null>( null );
		const detailsLoading = ref( false );
		const detailsError = ref( '' );

		async function onDetailsToggle( isOpen: boolean ) {
			if ( !isOpen || detailsHtml.value !== null || detailsLoading.value ) {
				return;
			}

			detailsLoading.value = true;
			detailsError.value = '';
			try {
				const { details } = await api.getInteractionDetails(
					props.interaction.job_id,
					props.interaction.id
				);
				const ansiUp = new AnsiUp();
				// Don't generate links for URLs in the details
				ansiUp.url_allowlist = {};
				detailsHtml.value = ansiUp.ansi_to_html( details );
			} catch ( e ) {
				detailsError.value = `Could not get the details: ${ e }`;
			} finally {
				detailsLoading.value = false;
			}
		}

		// Notify when a new interaction arrives, even if this component instance
		// is reused and does not remount.
		watch(
			() => props.interaction.id,
			() => {
				detailsHtml.value = null;
				detailsError.value = '';
				notifications.notifyUser( props.interaction );
			},
			{ immediate: true }
		);

		const renderedPromptHtml = computed( () => {
			const prompt = props.interaction.prompt;
			const onCurrentJobPage = route.name === 'job' &&
				String( route.params.jobId ) === String( props.interaction.job_id );

			if (
				props.interaction.type !== 'choices' ||
				onCurrentJobPage ||
				hasDetails.value
			) {
				return linkifyStr( prompt, linkifyOptions );
			}

			const jobLogHref = router.resolve( {
				name: 'job',
				params: { jobId: props.interaction.job_id },
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
			renderedPromptHtml,
			hasDetails,
			detailsHtml,
			detailsLoading,
			detailsError,
			onDetailsToggle
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

	&__details {
		margin-bottom: @spacing-50;

		&__text {
			font-family: 'Courier New', Courier, monospace;
			font-size: 0.875rem;
			max-height: 60vh;
			overflow: auto;
			padding: @spacing-50;
			border: @border-base;
			background-color: @background-color-base;
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
