# 为 Carla 做出贡献

Carla 团队很高兴接受任何愿意合作的人的贡献。根据贡献者的能力，可以采用不同的方式为项目做出贡献。该团队将尽最大努力将贡献成功整合到 Carla 中。

看看吧，不要犹豫！

*   [__报告错误__](#report-bugs)  
*   [__请求功能__](#request-features)  
*   [__代码贡献__](#code-contributions)  
	*   [了解虚幻引擎](#learn-about-unreal-engine)  
	*   [开始之前](#before-getting-started)  
	*   [编码标准](#coding-standard)  
	*   [提交](#submission)  
	*   [清单](#checklist)  
*   [__艺术贡献__](#art-contributions)  
*   [__文档贡献__](#docs-contributions)  

---
## 报告错误

可以在 GitHub 上的 [issue 部分][issueslink] 报告问题。在报告新错误之前，请务必进行一些检查。

__1. 检查是否已报告问题。__ 在 GitHub 上的同一问题部分查找它。

__2. 阅读文档。__ 确保问题是一个错误，而不是对 Carla 应该如何工作的误解。阅读 [文档][docslink] 中与该问题相关的页面并查看 [常见问题解答][faqlink] 页面。

[issueslink]: https://github.com/carla-simulator/carla/issues
[docslink]: http://carla.readthedocs.io
[faqlink]: build_faq.md

---
## 请求功能

新功能的想法也是一种很好的贡献方式。任何可以改善用户体验的建议都可以在 [此处][frlink] 的相应 GitHub 部分提交。

[frlink]: https://github.com/carla-simulator/carla/issues?q=is%3Aissue+is%3Aopen+label%3A%22feature+request%22+sort%3Acomments-desc

---
## 代码贡献

在开始动手编码之前，请查看 [issue 板块][issueboard] 以了解团队已经在做什么，以避免重叠。如有疑问或讨论如何继续，请联系我们之一（或发送电子邮件至 <carla.simulator@gmail.com>）。


为了开始工作，请 [创建 Carla 存储库的分叉](https://docs.github.com/en/enterprise/2.13/user/articles/fork-a-repo) ，并将所述分叉克隆到您的计算机中。请记住使您的分叉与原始存储库 [保持同步](https://docs.github.com/en/enterprise/2.13/user/articles/syncing-a-fork) 。

[issueboard]: https://github.com/carla-simulator/carla/issues

### 了解虚幻引擎

有关使用虚幻引擎 4 进行 C++ 编程的基本介绍，请参阅 Unreal 的 [C++ 编程教程][ue4tutorials]。网上还有其他选项，其中一些不是免费的。[Udemy 的虚幻 C++ 课程][ue4course]非常完整，并且通常提供价格非常实惠的优惠。

[ue4tutorials]: https://docs.unrealengine.com/latest/INT/Programming/Tutorials/
[ue4course]: https://www.udemy.com/unrealcourse/

### 开始之前

查看 [Carla 设计](index.md)<!-- @todo -->文档，了解组成 Carla 的不同模块。选择最合适的一个来容纳新功能。如果在此过程中出现任何疑问， 请随时联系 [Discord 服务器](https://discord.com/invite/8kqACuC) 中的团队。

### 编码标准

提交新代码时 遵循当前的  [编码标准](cont_coding_standard.md)。

### 提交

贡献和新功能不会直接合并到 `master` 分支，而是合并到名为 `dev` 的中间分支。这种 [Gitflow](https://nvie.com/posts/a-successful-git-branching-model/) 分支模型可以更轻松地维护稳定的主分支。该模型需要特定的贡献工作流程。

*   始终让您的 `dev` 分支更新最新的更改。
*   从名为 `username/name_of_the_contribution` 的 `dev` 子分支中开发贡献。
*   贡献准备就绪后，从您的分支向 `dev` 提交拉取请求（pull-request）。填写描述时尽量具有描述性。请注意，在合并之前，新代码需要通过一些检查。检查由持续集成系统自动运行。如果检查成功，将会出现绿色勾号。如果有红色标记，请相应更正代码。

一旦贡献被合并到 `dev` 中，就可以与其余新功能一起进行测试。到下一个版本时，该 `dev` 分支将合并到 `master`，并且贡献将可用并公布。

### 清单  

*   [ ] 您的分支已与 `dev` 分支保持同步，并通过最新更改进行了测试。
*   [ ] 如有必要，扩展 README/文档。
*   [ ] 代码编译正确。
*   [ ] 所有测试均通过 `make check`。

---
## 艺术贡献

艺术贡献包括车辆、行人、地图或 Carla 中使用的任何其他类型的资产。这些存储在 BitBucket 存储库中，该存储库有一些帐户空间限制。出于上述原因，贡献者必须与 Carla 团队联系，并要求他们在内容存储库上为贡献创建一个分支。

__1. 创建BitBucket账户。__ 访问 [Bitbucket 页面](https://bitbucket.org).  

__2. 联系艺术团队以获取内容存储库的访问权限。__ 加入 [Discord 服务器](https://discord.com/invite/8kqACuC)。转至 __贡献者__ 频道并请求访问内容存储库。 

__3. 将为每个贡献者创建一个分支。__ 该分支将被命名为 `contributors/contributor_name`。该用户所做的所有贡献都应在相应的分支中做出。

__4. 构建 Carla。__ 为了做出贡献，Carla 构建是必要的。按照说明在 [Linux](https://carla.readthedocs.io/en/latest/build_linux/) 或 [Windows](https://carla.readthedocs.io/en/latest/build_windows/) 中构建。

__5. 下载内容存储库。__ 按照说明更新 [此处](https://carla.readthedocs.io/en/latest/build_update/#get-development-assets) 的内容。

__6. 更新分支以与 master 同步。__ 分支应始终使用 master 中的最新更改进行更新。

__7. 上传贡献。__ 进行相应的更改并将分支推送到 origin。  

__8. 等待美术组审核。__ 一旦贡献上传，团队将检查与 master 合并的一切是否准备好。


---
## 文档贡献

如果某些文档缺失、模糊或不精确，则可以像任何其他错误一样进行报告（请阅读上一节有关 [如何报告错误](#report-bugs)）。但是，用户可以通过编写文档来做出贡献。


该文档是用 [Markdown](https://www.markdownguide.org/) 和 HTML 标签混合编写的，还有一些额外的 CSS 代码用于表格或 [town slider](https://carla.readthedocs.io/en/latest/core_map/#carla-maps) 等功能。按照以下步骤开始编写文档。

!!! 重要
    要提交文档贡献，请遵循上面 [代码贡献](#submission) 中解释的相同工作流程。总而言之，贡献是在来自 `dev` 分支的子分支中做出的，并合并到该分支。

__1. 从源代码构建 Carla。__ 按照文档中的步骤在 [Linux](build_linux.md) 或 [Windows](build_windows.md) 上进行构建。


__2. 安装 [MkDocs](http://www.mkdocs.org/).__ MkDocs 是一个用于构建文档的静态站点生成器。

```sh
sudo pip install mkdocs
```

__3. 可视化文档。__ 在主 Carla 文件夹中，运行以下命令并单击终端中显示的链接 (http://127.0.0.1:8000) 以打开文档的本地可视化。

```sh
mkdocs serve
```
__4. 创建git分支。__ 创建新分支时，请确保位于`dev`分支中（更新到最新更改）。

```sh
git checkout -b <contributor_name>/<branch_name>
```

__5. 编写文档。__ 按照 [文档标准](cont_doc_standard.md) 页面中的指南编辑文件。

__6. 提交更改。__ 在 GitHub 存储库中创建拉取请求，并添加一个建议的审阅者。填写拉取请求描述时，请尝试尽可能具有描述性。

__7. 等待审核。__ 团队将检查合并或需要进行任何更改是否已准备好。

!!! 警告
    本地存储库必须使用 `dev` 分支中的最新更新进行更新。