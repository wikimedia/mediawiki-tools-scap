import { fileURLToPath, URL } from 'node:url';

import { visualizer } from 'rollup-plugin-visualizer';
import { defineConfig } from 'vite';
import vue from '@vitejs/plugin-vue';
import vuetify from 'vite-plugin-vuetify';

// https://vitejs.dev/config/
export default defineConfig( {
	plugins: [
		visualizer( {
			emitTile: true,
			filename: 'dist/chunk-stats.html'
		} ),
		vue(),
		vuetify()
	],
	build: {
		rollupOptions: {
			output: {
				manualChunks( id ) {
					// Create separate chunks for some large dependencies
					if (
						id.includes( '@vue' ) ||
						id.includes( 'pinia' ) ||
						id.includes( 'vue-router' )
					) {
						return '@vue';
					}
					if ( id.includes( '@wikimedia' ) ) {
						return '@wikimedia';
					}
				}
			}
		}
	},
	resolve: {
		alias: {
			'@': fileURLToPath( new URL( './src', import.meta.url ) )
		}
	},
	test: {
		environment: 'jsdom',
		server: {
			deps: {
				inline: [
					'vuetify',
					'@wikimedia/codex'
				]
			}
		}
	}
} );
