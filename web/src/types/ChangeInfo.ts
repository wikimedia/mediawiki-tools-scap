export interface CommitLink {
	href: string
	text: string
}

export default interface ChangeInfo {
	linkifiedCommitMsg: Array<string | CommitLink>
	subject: string
	project: string
	branch: string
	number: number
	url: string
	repoQueryUrl: string
	branchQueryUrl: string
}