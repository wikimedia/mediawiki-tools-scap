<template>
	<cdx-menu-button
		v-model:selected="selection"
		:menu-items="menuItems"
		aria-label="Choose an option"
		@update:selected="onSelect"
	>
		<cdx-icon :icon="cdxIconUserActive" />
	</cdx-menu-button>
</template>

<script>
import { defineComponent, ref, onMounted } from 'vue';
import { useRouter } from 'vue-router';
import { CdxMenuButton, CdxIcon } from '@wikimedia/codex';
import { cdxIconUserActive, cdxIconLogOut } from '@wikimedia/codex-icons';
import useApi from '../api';

const menuItems = ref( [] );

export default defineComponent( {
	name: 'UserMenu',
	components: { CdxMenuButton, CdxIcon },
	setup() {
		const api = useApi();
		const router = useRouter();
		const adminUrl = router.resolve( { name: 'admin' } ).href;
		const selection = ref( null );

		async function onSelect( operation ) {
			selection.value = null;

			if ( operation === 'signout' ) {
				const SSOLogoutUrl = await api.logout();
				document.location = SSOLogoutUrl;
				return;
			}
		}

		async function setupMenuItems() {

			const resp = await api.whoami();

			if ( !resp.user ) {
				// The user is not signed in.
				menuItems.value = [ { label: 'Sign In', value: 'signin', url: resp.loginUrl } ];
				return;
			}

			// Signed-in user

			const items = [ ];

			// The menuitem for the user doesn't do anything, so make it disabled.
			// In the future it could send the user to a user settings page.
			items.push( { label: `Welcome ${ resp.user }!`, value: 'user', disabled: true, boldLabel: true } );
			if ( resp.isAdmin ) {
				items.push( { label: 'Admin operations', value: 'admin', url: adminUrl } );
			}
			items.push( { label: 'Sign Out', value: 'signout', icon: cdxIconLogOut } );

			menuItems.value = items;
		}

		onMounted( () => {
			setupMenuItems();
		} );

		return {
			selection,
			menuItems,
			onSelect,
			cdxIconUserActive
		};
	}
} );
</script>
