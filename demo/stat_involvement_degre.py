import requests

# 替换为你的个人访问令牌（可选，但推荐）
TOKEN = 'your_personal_access_token'

headers = {
    'Authorization': f'token {TOKEN}',
    'Accept': 'application/vnd.github.v3+json'
}

owner = 'owner_name'  # 替换为仓库所有者
repo = 'repo_name'    # 替换为仓库名称

# 初始化统计字典
issue_counts = {}
comment_counts = {}

page = 1
while True:
    url = f'https://api.github.com/repos/{owner}/{repo}/issues?state=all&per_page=100&page={page}'
    response = requests.get(url, headers=headers)
    issues = response.json()

    if not issues:
        break

    for issue in issues:
        # 过滤掉 Pull Requests
        if 'pull_request' in issue:
            continue

        # 统计提问者
        user = issue['user']['login']
        issue_counts[user] = issue_counts.get(user, 0) + 1

        # 获取 Issue 的评论
        comments_url = issue['comments_url']
        comments_response = requests.get(comments_url, headers=headers)
        comments = comments_response.json()

        for comment in comments:
            commenter = comment['user']['login']
            comment_counts[commenter] = comment_counts.get(commenter, 0) + 1

    page += 1

# 输出结果
print("提问次数：")
for user, count in issue_counts.items():
    print(f"{user}: {count}")

print("\n回答次数：")
for user, count in comment_counts.items():
    print(f"{user}: {count}")