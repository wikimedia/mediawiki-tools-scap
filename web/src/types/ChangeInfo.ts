import LinkSegment from './LinkSegment';

export default interface ChangeInfo {
	linkifiedCommitMsg: Array<string | LinkSegment>
	subject: string
	project: string
	branch: string
	number: number
	url: string
	repoQueryUrl: string
	branchQueryUrl: string
}