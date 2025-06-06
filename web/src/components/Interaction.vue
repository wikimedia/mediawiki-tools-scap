<template>
	<div class="interaction">
		<div
			class="interaction__prompt"
			v-html="linkifiedPrompt" />
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
		const notifications = notificationsStore();
		let userNotification = null;

		function choiceSelected( code: string ) {
			api.respondInteraction(
				props.interaction.job_id,
				props.interaction.id,
				code
			);
			if ( userNotification ) {
				userNotification.close();
			}
		}

		// Lifecycle hooks
		onMounted( () => {
			if (
				notifications.userShouldBeNotified( props.interaction ) &&
				// Keep in sync with the prompt message in scap/backport.py#Backport._do_backport
				!props.interaction.prompt.includes( 'Backport the changes?' )
			) {
				userNotification = new Notification( 'SpiderPig needs you!', {
					body: `Job ${ props.interaction.job_id } requires user input`,
					requireInteraction: true
				} );
				notifications.rememberNotified( props.interaction.id );
			}
		} );

		const linkifiedPrompt = computed( () => linkifyStr( props.interaction.prompt,
			{
				target: '_blank',
				className: 'interaction__prompt__link',
				defaultProtocol: 'https'
			}
		) );

		return {
			choiceSelected,
			linkifiedPrompt
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
		margin-top: @spacing-100;

		button:first-child {
			margin-right: @spacing-25;
		}
	}
}

</style>
