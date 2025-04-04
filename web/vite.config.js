import { fileURLToPath, URL } from 'node:url';

import { visualizer } from 'rollup-plugin-visualizer';
import { defineConfig, splitVendorChunkPlugin } from 'vite';
import vue from '@vitejs/plugin-vue';
import vuetify from 'vite-plugin-vuetify';

// https://vitejs.dev/config/
export default defineConfig( {
	plugins: [
		splitVendorChunkPlugin(),
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
					if ( id.includes( '@xterm' ) ) {
						return '@xterm';
					}
				}
			}
		}
	},
	resolve: {
		alias: {
			'@': fileURLToPath( new URL( './src', import.meta.url ) )
		}
	}
} );
