import globals from 'globals';
import tsPlugin from '@typescript-eslint/eslint-plugin';
import tsParser from '@typescript-eslint/parser';
import vue from 'eslint-plugin-vue';
import vueParser from 'vue-eslint-parser';

export default [
	{
		ignores: [
			'dist/',
			'src/mocks/fakeBackportScripts/',
			'src/mocks/fakeBackport.mjs',
			'src/mocks/fakeApiServer.js',
			'node_modules/',
			'.vite/'
		]
	},
	{
		files: ['**/*.{js,mjs,cjs,jsx}'],
		languageOptions: {
			ecmaVersion: 2022,
			sourceType: 'module',
			globals: globals.browser
		},
		rules: {
			'no-unused-vars': ['error', {
				argsIgnorePattern: '^_'
			}]
		}
	},
	{
		files: ['**/*.vue'],
		languageOptions: {
			parser: vueParser,
			parserOptions: {
				parser: tsParser,
				extraFileExtensions: ['.vue'],
				sourceType: 'module'
			},
			globals: globals.browser
		},
		plugins: {
			vue,
			'@typescript-eslint': tsPlugin
		},
		rules: {
			...vue.configs['essential'].rules,
			'vue/component-name-in-template-casing': ['error', 'kebab-case'],
			'vue/custom-event-name-casing': ['error', 'kebab-case'],
			'vue/no-undef-components': ['error', {
				ignorePatterns: [
					'router-link',
					'router-view',
					'v-*'
				]
			}],
			'vue/max-attributes-per-line': ['warn', {
				singleline: 4
			}],
			'vue/no-parsing-error': ['error', {
				'unexpected-character-in-attribute-name': false
			}],
			'vue/valid-v-slot': ['error', {
				allowModifiers: true
			}]
		}
	}
];
