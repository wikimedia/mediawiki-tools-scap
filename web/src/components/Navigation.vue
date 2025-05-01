<template>
	<v-app-bar
		class="sp-navigation"
		color="surface-variant"
		flat
		:elevation="2">
		<v-img
			class="sp-logo"
			max-width="64"
			:src="spLogo"
			alt="Spiderpig logo" />

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
			</v-tabs>
		</template>
	</v-app-bar>
</template>

<script lang="ts">
import { VAppBar, VAppBarTitle } from 'vuetify/components/VAppBar';
import { VBtn } from 'vuetify/components/VBtn';
import { VImg } from 'vuetify/components/VImg';
import { VSpacer } from 'vuetify/components/VGrid';
import { VTab, VTabs } from 'vuetify/components/VTabs';
import UserMenu from './UserMenu.vue';
import { CdxIcon } from '@wikimedia/codex';
import { cdxIconLinkExternal } from '@wikimedia/codex-icons';
import spLogo from '../assets/spiderpig.png';

export default {
	name: 'SpNavigation',
	components: {
		UserMenu,
		CdxIcon,
		VAppBar,
		VAppBarTitle,
		VBtn,
		VImg,
		VSpacer,
		VTab,
		VTabs
	},

	setup() {
		return {
			cdxIconLinkExternal,
			spLogo
		};
	}
};
</script>

<style lang="less">
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import ( reference ) '@wikimedia/codex/mixins/link.less';

.sp-navigation {
	.v-toolbar__content {
		overflow: inherit;
	}

	.v-img, .v-btn {
		margin-left: @spacing-100;
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
}
</style>
