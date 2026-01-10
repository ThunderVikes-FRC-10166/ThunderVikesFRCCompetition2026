# ThunderVikes FRC Robot Code – How Students Should Work on This Repo

This document explains **how you, as a student**, should work on this code in a way that:

- Keeps the `main` branch stable
- Lets everyone review each other’s code
- Uses **GitHub Desktop** (no command line needed)
- Matches how real software teams work

Read this slowly. You don’t need to memorize it — you can come back to it any time.

---

## Big picture: How we work on code as a team

We follow this basic workflow:

1. **You do NOT code directly on `main`.**
2. You create your **own branch** (a “copy lane” of the code).
3. You make changes on your branch.
4. You **push** your changes to GitHub.
5. You open a **Pull Request (PR)**.
6. Other students and mentors **review** your PR.
7. After review, your changes are **merged** into `main`.

Think of `main` as the **“official robot code”** lane.
Branches are **“sandbox lanes”** where you can experiment safely without breaking everyone else.

---

## Step 1: Set up GitHub and GitHub Desktop

### 1.1 – Make a GitHub account

If you don’t already have one:

- Go to: https://github.com
- Click **Sign up**
- Follow the steps to create an account  
  (Use an email you can access, and pick a username you’re okay using for school/college too.)

Tell a mentor your GitHub username so they can add you to the team organization.

---

### 1.2 – Install GitHub Desktop

We use **GitHub Desktop**, a simple app that lets you work with Git without using the terminal.

- Go to: https://desktop.github.com
- Download and install GitHub Desktop
- Open GitHub Desktop
- Log in using your GitHub account

---

## Step 2: Get the robot code onto your computer

You only do this once per computer.

1. Go to the **team’s GitHub organization** in your browser.
2. Click on the **robot code repository** (for example: `ThunderVikesFRCCompetition2026`).
3. On the repo page, find the **green “Code” button**.
4. Click it, then click **“Open with GitHub Desktop”**.
   - If your browser asks “Open GitHub Desktop?”, click **Yes** or **Open**.
5. GitHub Desktop will open and ask where to save the repository.
   - Choose a folder that is **NOT on OneDrive** if possible (for example: `C:\Users\YourName\Documents\GitHub\`).
6. Click **Clone**.

Now you have a **local copy** of the robot code on your computer.

---

## Step 3: Never code directly on `main`

When you open the repo in GitHub Desktop, look at the top:

- There’s a dropdown that shows the current **branch**.
- It will usually say `main` when you first clone.

**Important rule:**  
You should **never** make changes directly on `main`.

`main` is the branch that represents “this is what we would deploy to the robot.”

Instead, you will:

- Create a **new branch** for any feature or fix you work on.

---

## Step 4: Create your own branch

Think of a branch as your own **working lane**.

### 4.1 – Naming your branch

Use a name that describes what you’re doing. For example:

- `feature/drive-code`
- `feature/intake`
- `bugfix/joystick-deadzone`
- `studentname/learning-magicbot`

### 4.2 – Creating the branch in GitHub Desktop

1. Open **GitHub Desktop**.
2. Make sure you have the correct repo selected (top left).
3. At the top, next to “Current branch”, click the **branch dropdown**.
4. Click **“New branch”**.
5. Enter a branch name (see examples above).
6. Make sure “Create branch based on” is set to **`main`**.
7. Click **Create branch**.
8. GitHub Desktop will switch you to that new branch automatically.

Now any code changes you make will be on **your branch**, not on `main`.

---

## Step 5: Edit the code in your branch

Now you actually write code.

1. In GitHub Desktop, click **“Open in your editor”** (e.g., PyCharm or VS Code).
2. Make sure your editor is opening the folder for the same repo.
3. Edit files as needed (following team coding standards and instructions).
4. Save your changes in your editor.

GitHub Desktop will automatically notice the changes.

---

## Step 6: Commit your changes (save your work with a message)

A **commit** is like a “save point” with a message describing what you did.

### 6.1 – View your changes

1. Go back to **GitHub Desktop**.
2. On the left, you’ll see a list of files that changed.
3. Click each file to see what you added/removed (red = removed, green = added).

### 6.2 – Write a commit message

On the bottom left:

- In the **“Summary”** box, type a short description.
  - Example: `Add basic drive code using left joystick`
- The **“Description”** box is optional but useful for more details.

### 6.3 – Commit

1. Check the box next to the files you want to commit (usually all of them).
2. Click the **“Commit to [your-branch-name]”** button.

You’ve now created a commit on your branch.  
This commit is still **only on your computer**, not on GitHub yet.

---

## Step 7: Push your branch to GitHub

To share your work with the team, you need to **push** your branch.

1. After you commit, look at the top of GitHub Desktop.
2. There will be a button that says **“Push origin”** (or something similar).
3. Click **“Push origin”**.

Now:

- Your branch (and its commits) are uploaded to GitHub.
- Other team members can see your branch on the website.

---

## Step 8: Make a Pull Request (PR)

A **Pull Request (PR)** is how you ask to merge your branch into `main`.

You’re basically saying:  
“Hey team, I’ve made changes on this branch. Please review them and, if they’re good, merge them into the main robot code.”

### 8.1 – Open the PR from GitHub Desktop

1. After pushing your branch, GitHub Desktop may show a banner:
   - “Create Pull Request”
   - If you see that, click **“Create Pull Request”**.
2. This will open your browser to GitHub’s PR page.

If it doesn’t show:

- Go to the repo on GitHub in your browser.
- You may see a message like “Compare & pull request” for your branch.
- Click **“Compare & pull request”**.

### 8.2 – Fill out the PR form

On the PR page:

- **Title**: Short description of what this change does.
  - Example: `Add basic tank drive control`
- **Description**: Explain:
  - What you changed
  - Why you changed it
  - Anything reviewers should pay attention to
  - How you tested it (simulation, test board, robot, etc.)

You can also tag mentors or leads (if you know how), but it’s not required.

When done, click **“Create pull request”**.

Now your PR is open. Other people can see the changes you made.

---

## Step 9: Code review – what happens after you open a PR

Once you open a PR, other students and mentors will:

- Read your code
- Leave comments
- Ask questions
- Suggest improvements
- Approve or request changes

This is **normal** and **good**.  
Review is not criticism; it’s about making the code safe, clear, and correct for the robot.

### 9.1 – Making changes after review

If someone asks for changes:

1. Go back to your **branch** in GitHub Desktop.
2. Open the code in your editor.
3. Make the requested changes.
4. Save the files.
5. In GitHub Desktop:
   - Commit the new changes with another message.
   - Push the branch again.

The PR will automatically update with your new commits.  
You do **not** make a new PR for each change.

---

## Step 10: Merging your PR into main

When mentors / leads are happy with your PR:

- They will click **“Merge”** on GitHub.
- Your branch’s changes will be added into `main`.

After that, your feature is officially part of the robot code.

Sometimes we may:

- Squash commits (combine them)
- Rebase (advanced – mentors handle this)

You don’t need to worry about the details right now.

---

## Step 11: Keeping your local main up to date

From time to time, you should update your local `main` branch so you’re working on the latest code.

1. In GitHub Desktop, switch to the `main` branch:
   - Click the branch dropdown at the top.
   - Select `main`.
2. Click **“Fetch origin”** (top).
3. If there are new changes, it will show **“Pull origin”**.
4. Click **“Pull origin”** to download updates.

If you want to start a new branch for a new feature:

- Always base it on the **updated `main` branch**.

---

## Common questions

### Q: What if I accidentally change code on `main`?

- Don’t panic.
- Tell a mentor or programming lead.
- They will help you either:
  - Move the changes to a new branch, or
  - Fix the branch state.

### Q: How often should I commit?

- Commit **small, logical chunks**.
- Example:
  - After getting joystick input working → commit.
  - After getting motors to respond → another commit.
- Don’t wait until everything is done to commit.

### Q: What if my code is messy or not done yet?

- It’s okay to open a **draft PR** or note “work in progress”.
- Getting feedback early is better than waiting until the last minute.

---

## Quick summary (cheat sheet)

1. **Clone** the repo using GitHub Desktop.
2. **Create a new branch** (never code directly on `main`).
3. **Edit code** in your editor.
4. **Commit** changes in GitHub Desktop with a clear message.
5. **Push** your branch to GitHub.
6. **Open a Pull Request** on GitHub.
7. Respond to **review comments** and push updates.
8. Mentors/leads **merge** into `main` when ready.
9. Keep your local `main` **updated** by pulling regularly.

---

If you’re ever unsure, **ask a mentor or an experienced student**.  
You are not expected to figure all of this out alone the first time. This process is meant to help the whole team work together safely on the code that runs the robot.
