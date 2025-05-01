<template>
	<v-menu>
		<template #activator="{ props }">
			<v-btn color="surface-variant" v-bind="props">
				<cdx-icon :icon="cdxIconUserActive" />
			</v-btn>
		</template>
		<v-list>
			<v-list-item
				v-for="( item, i ) in menuItems"
				:key="i"
				v-bind="item"
				@click="onSelect( item.value )"
			/>
		</v-list>
	</v-menu>
</template>

<script>
import { defineComponent, ref, onMounted } from 'vue';
import { useRouter } from 'vue-router';
import { CdxIcon } from '@wikimedia/codex';
import { cdxIconUserActive } from '@wikimedia/codex-icons';
import useApi from '../api';

const menuItems = ref( [] );

export default defineComponent( {
	name: 'UserMenu',
	components: { CdxIcon },
	setup() {
		const api = useApi();
		const router = useRouter();
		const adminUrl = router.resolve( { name: 'admin' } ).href;

		async function onSelect( operation ) {
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
				menuItems.value = [ { title: 'Sign In', value: 'signin', href: resp.loginUrl } ];
				return;
			}

			// Signed-in user

			const items = [ ];

			// The menuitem for the user doesn't do anything, so make it disabled.
			// In the future it could send the user to a user settings page.
			items.push( { title: `Welcome ${ resp.user }!`, value: 'user', link: false, color: 'primary' } );
			if ( resp.isAdmin ) {
				items.push( { title: 'Admin operations', value: 'admin', href: adminUrl } );
			}
			items.push( { title: 'Sign Out', value: 'signout', 'append-icon': 'mdi-logout' } );

			menuItems.value = items;
		}

		onMounted( () => {
			setupMenuItems();
		} );

		return {
			menuItems,
			onSelect,
			cdxIconUserActive
		};
	}
} );
</script>
