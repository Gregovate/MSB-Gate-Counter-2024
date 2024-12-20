import re

# Paths to the files
source_file = "src/main.cpp"
changelog_file = "docs/changelog.md"

def extract_changelog_and_version(source_path):
    with open(source_path, "r") as file:
        content = file.read()
    
    # Extract changelog entries from the comment section
    changelog_match = re.search(r"/\*[\s\S]*?Changelog([\s\S]*?)\*/", content)
    changelog = []
    if changelog_match:
        changelog_lines = changelog_match.group(1).strip().split("\n")
        for line in changelog_lines:
            match = re.match(r"(\d{2}\.\d{2}\.\d{2}\.\d+)\s+-\s+(.*)", line.strip())
            if match:
                version, description = match.groups()
                changelog.append((version, description))
    
    # Extract current FWVersion
    version_match = re.search(r'#define\s+FWVersion\s+"(\d{2}\.\d{2}\.\d{2}\.\d+)"', content)
    current_version = version_match.group(1) if version_match else None

    return changelog, current_version

def update_fw_version(source_path, new_version):
    with open(source_path, "r") as file:
        content = file.read()
    
    # Replace the FWVersion definition
    updated_content = re.sub(
        r'#define\s+FWVersion\s+"(\d{2}\.\d{2}\.\d{2}\.\d+)"',
        f'#define FWVersion "{new_version}"',
        content
    )

    with open(source_path, "w") as file:
        file.write(updated_content)
    print(f"FWVersion updated to {new_version} in {source_path}")

def update_changelog_md(changelog, changelog_path):
    with open(changelog_path, "r") as file:
        existing_changelog = file.read()
    
    # Add new entries to the changelog if they don't already exist
    new_entries = []
    for version, description in changelog:
        if version not in existing_changelog:
            new_entries.append(f"### [{version}] - YYYY-MM-DD\n- {description}\n")
    
    if new_entries:
        updated_changelog = "\n".join(new_entries) + "\n" + existing_changelog
        with open(changelog_path, "w") as file:
            file.write(updated_changelog)
        print(f"{len(new_entries)} new entries added to changelog.md")
    else:
        print("No new entries to add to changelog.md")

# Workflow
changelog, fw_version = extract_changelog_and_version(source_file)

if changelog:
    latest_version = changelog[0][0]  # The most recent version is the first in the changelog
    if fw_version != latest_version:
        print(f"Updating FWVersion from {fw_version} to {latest_version}")
        update_fw_version(source_file, latest_version)
    else:
        print("FWVersion is already up-to-date")

    update_changelog_md(changelog, changelog_file)
else:
    print("No changelog entries found in the source file.")
