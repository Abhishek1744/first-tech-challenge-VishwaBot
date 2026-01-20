# Git Cheat Sheet (For Developers)

This document provides a quick reference to commonly used Git commands and explains **what they do** and **when to use them**. Keep this file in the project to help all developers follow a consistent Git workflow.

---

## 1. Basic Git Commands

### Initialize a repository
```bash
git init
```
Creates a new Git repository in the current directory.

---

### Check repository status
```bash
git status
```
Shows modified, staged, and untracked files.

---

### Add files to staging area
```bash
git add file.txt
git add .
```
Prepares files to be included in the next commit.

---

### Commit changes
```bash
git commit -m "commit message"
```
Saves a snapshot of staged changes to the repository history.

---

## 2. Remote Repository (GitHub)

### Add remote repository
```bash
git remote add origin <repo-url>
```
Connects local repository to a remote GitHub repository.

---

### Push changes to remote
```bash
git push origin main
```
Uploads commits to GitHub.

First push only:
```bash
git push -u origin main
```

---

### Pull latest changes
```bash
git pull origin main
```
Fetches and merges changes from the remote repository.

---

### Clone a repository
```bash
git clone <repo-url>
```
Creates a local copy of an existing remote repository.

---

## 3. Branching

### List branches
```bash
git branch
```

---

### Create a new branch
```bash
git branch feature-branch
```

---

### Switch branch
```bash
git checkout feature-branch
```
OR
```bash
git switch feature-branch
```

---

### Create and switch branch
```bash
git checkout -b feature-branch
```

---

### Merge a branch
```bash
git checkout main
git merge feature-branch
```
Merges feature branch into main.

---

## 4. Undo & Fix Mistakes

### Unstage a file
```bash
git restore --staged file.txt
```

---

### Discard local changes
```bash
git restore file.txt
```

---

### Amend last commit message
```bash
git commit --amend
```

---

### Revert a commit (safe)
```bash
git revert <commit-hash>
```
Creates a new commit that undoes a previous commit.

---

### Reset last commit (dangerous)
```bash
git reset --hard HEAD~1
```
Removes the last commit completely (use carefully).

---

## 5. Logs & History

### View commit history
```bash
git log
```

Compact view:
```bash
git log --oneline --graph --all
```

---

## 6. Stash (Temporary Save)

### Save uncommitted changes
```bash
git stash
```

---

### Restore stashed changes
```bash
git stash pop
```

---

## 7. Tags (Releases)

```bash
git tag v1.0
git push origin v1.0
```
Used to mark release versions.

---

## 8. Recommended Team Workflow

1. Pull latest changes from main
2. Create a feature branch
3. Commit changes frequently
4. Push branch to GitHub
5. Open a Pull Request
6. Get review (collaborators)
7. Merge to main (owner)

---

## 9. Important Rules

- Do not push directly to `main` (except owner if allowed)
- Write clear commit messages
- Pull before pushing
- Keep commits small and meaningful

---

**End of Git Cheat Sheet**

