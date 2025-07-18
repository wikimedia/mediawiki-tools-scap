<template>
	<v-app-bar
		class="sp-navigation"
		color="surface-variant"
		flat
		:elevation="2"
	>
		<router-link to="/">
			<img
				class="sp-logo"
				:src="spLogo"
				alt="Spiderpig logo"
			>
		</router-link>

		<v-app-bar-title class="sp-banner">
			<router-link to="/">
				SPIDER PIG
			</router-link>
		</v-app-bar-title>

		<v-spacer />

		<v-btn>
			<a href="https://logstash.wikimedia.org/app/dashboards#/view/mediawiki-errors" target="_blank">
				Logstash<cdx-icon :icon="cdxIconLinkExternal" />
			</a>
		</v-btn>
		<v-btn icon>
			<user-menu />
		</v-btn>

		<template #extension>
			<v-tabs fixed>
				<v-tab to="/mediawiki/backport">
					MediaWiki Backport
				</v-tab>
				<v-tab to="/mediawiki/train">
					MediaWiki Train
				</v-tab>
				<v-tab to="/mediawiki/logs">
					MediaWiki Error Logs
					&nbsp;
					<error-logs-count />
				</v-tab>
			</v-tabs>
		</template>
	</v-app-bar>
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import { VAppBar, VAppBarTitle } from 'vuetify/components/VAppBar';
import { VBtn } from 'vuetify/components/VBtn';
import { VSpacer } from 'vuetify/components/VGrid';
import { VTab, VTabs } from 'vuetify/components/VTabs';
import ErrorLogsCount from './ErrorLogsCount.vue';
import UserMenu from './UserMenu.vue';
import { CdxIcon } from '@wikimedia/codex';
import { cdxIconLinkExternal } from '@wikimedia/codex-icons';
import spLogo from '../assets/spiderpig.png';

export default defineComponent( {
	name: 'SpNavigation',
	components: {
		UserMenu,
		CdxIcon,
		VAppBar,
		VAppBarTitle,
		VBtn,
		VSpacer,
		VTab,
		VTabs,
		ErrorLogsCount
	},

	setup() {
		return {
			cdxIconLinkExternal,
			spLogo
		};
	}
} );
</script>

<style lang="less">
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import ( reference ) '@wikimedia/codex/mixins/link.less';

.sp-navigation {
	.v-toolbar__content {
		overflow: inherit;
	}

	.sp-logo {
		max-width: 64px;
	}

	a {
		color: inherit;
		text-decoration: none;
		font-weight: @font-weight-bold;
	}

	.sp-banner {
		font-family: Montserrat, Avenir, Corbel, 'URW Gothic', source-sans-pro, sans-serif;
		font-size: @font-size-xxx-large;
		font-style: normal;
		line-height: normal;
		letter-spacing: 0.4px;
	}

	.cdx-icon {
		color: rgb(var(--v-theme-on-surface-variant));
	}

	.cdx-toggle-button {
		&.cdx-toggle-button--toggled-on:not(:enabled:hover) {
			.cdx-icon {
				color: rgb(var(--v-theme-on-surface-variant));
			}
		}
	}
}
</style>
