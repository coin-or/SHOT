// SHOT tokens use the root theme; Furo's accessible theme controls update body.
(() => {
  const synchronize = () => {
    const theme = document.body.dataset.theme;
    if (theme === 'light' || theme === 'dark') document.documentElement.dataset.theme = theme;
    else delete document.documentElement.dataset.theme;
  };
  synchronize();
  new MutationObserver(synchronize).observe(document.body, {attributes: true, attributeFilter: ['data-theme']});
})();

// Copy only the underlying source, never line numbers or button text.
async function copyText(button, text) {
  clearTimeout(button.copyResetTimer);
  try {
    await navigator.clipboard.writeText(text);
    button.dataset.state = 'copied';
    button.setAttribute('aria-label', 'Copied');
    button.title = 'Copied';
  } catch {
    button.dataset.state = 'failed';
    button.setAttribute('aria-label', 'Copy failed; try again');
    button.title = 'Copy failed; try again';
  }
  button.copyResetTimer = setTimeout(() => {
    delete button.dataset.state;
    button.setAttribute('aria-label', button.dataset.copyLabel);
    button.title = button.dataset.copyLabel;
  }, 1800);
}
function copyButton(text, label = 'Copy') {
  const button = document.createElement('button');
  button.type = 'button'; button.className = 'shot-copy';
  button.dataset.copyLabel = label;
  button.setAttribute('aria-label', label); button.title = label;
  button.innerHTML = `<svg viewBox="0 0 24 24" aria-hidden="true" focusable="false">
    <g class="copy-symbol"><rect x="8" y="8" width="12" height="13" rx="2"/><path d="M16 8V5a2 2 0 0 0-2-2H5a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h3"/></g>
    <path class="copy-success" d="m5 12 4 4L19 6"/>
    <path class="copy-failure" d="m6 6 12 12M18 6 6 18"/>
  </svg>`;
  button.addEventListener('click', () => copyText(button, text));
  return button;
}
document.querySelectorAll('article div.highlight').forEach(panel => {
  const code = panel.querySelector('pre');
  if (code) panel.append(copyButton(code.textContent.trimEnd(), 'Copy code'));
});
const siteToggle = document.querySelector('.shot-nav-toggle');
siteToggle?.addEventListener('click', () => {
  const open = siteToggle.getAttribute('aria-expanded') !== 'true';
  siteToggle.setAttribute('aria-expanded', String(open));
  document.querySelector('.shot-docs-header').classList.toggle('nav-open', open);
});

// Enhance the generated settings tables; without JS the full reference remains readable.
const settingsPage = document.querySelector('article #solver-settings');
if (settingsPage) {
  settingsPage.classList.add('settings-reference');
  const controls = document.createElement('div');
  controls.className = 'settings-controls';
  const label = document.createElement('label');
  label.textContent = 'Find a setting'; label.htmlFor = 'settings-search';
  const search = document.createElement('input');
  search.id = 'settings-search'; search.type = 'search';
  search.placeholder = 'Search names, descriptions, or values';
  const status = document.createElement('p');
  status.className = 'settings-status'; status.setAttribute('role', 'status');
  const categories = document.createElement('nav');
  categories.className = 'settings-categories'; categories.setAttribute('aria-label', 'Setting categories');
  controls.append(label, search, categories, status);
  settingsPage.querySelector('h1').after(controls);
  const groups = [];
  settingsPage.querySelectorAll(':scope > section').forEach(section => {
    const heading = section.querySelector(':scope > h2');
    if (!heading) return;
    const details = document.createElement('details');
    details.className = 'settings-group'; details.open = false;
    const summary = document.createElement('summary');
    summary.textContent = heading.childNodes[0].textContent;
    section.insertBefore(details, heading);
    details.append(summary);
    Array.from(section.childNodes).filter(node => node !== details && node !== heading).forEach(node => details.append(node));
    heading.remove();
    const link = document.createElement('a'); link.href = '#' + section.id; link.textContent = summary.textContent;
    link.className = 'shot-btn shot-btn--ghost';
    link.addEventListener('click', () => { search.value = ''; filter(); details.open = true; });
    categories.append(link);
    details.querySelectorAll('table').forEach(table => {
      table.querySelector('colgroup')?.remove();
      const header = document.createElement('th');
      header.scope = 'col'; header.className = 'settings-copy-column';
      const label = document.createElement('span'); label.className = 'visually-hidden';
      label.textContent = 'Copy setting'; header.append(label);
      table.querySelector('thead tr')?.append(header);
    });
    const rows = Array.from(details.querySelectorAll('tbody tr'));
    rows.forEach(row => {
      const cells = row.querySelectorAll('td');
      const name = cells[0]?.querySelector('strong')?.textContent;
      if (!name || cells.length < 3) return;
      const value = cells[2].textContent.trim();
      row.dataset.search = row.textContent.toLowerCase();
      const button = copyButton(`${name}=${value}`, `Copy ${name}=${value}`);
      const copyCell = document.createElement('td');
      copyCell.className = 'settings-copy-column'; copyCell.append(button); row.append(copyCell);
    });
    groups.push({details, section, rows});
  });
  function filter() {
    const terms = search.value.toLowerCase().trim().split(/\s+/).filter(Boolean);
    let count = 0;
    groups.forEach(({details, section, rows}) => {
      let visible = 0;
      rows.forEach(row => {
        row.hidden = !terms.every(term => (row.dataset.search || row.textContent.toLowerCase()).includes(term));
        if (!row.hidden) visible++;
      });
      section.hidden = visible === 0;
      if (terms.length && visible) details.open = true;
      details.querySelectorAll('section').forEach(sub => {
        const subRows = Array.from(sub.querySelectorAll('tbody tr'));
        sub.hidden = subRows.length > 0 && subRows.every(row => row.hidden);
      });
      count += visible;
    });
    status.textContent = `${count} settings${terms.length ? ' match your search' : ''}`;
  }
  search.addEventListener('input', filter);
  const revealHash = () => {
    const id = decodeURIComponent(location.hash.slice(1));
    const target = document.getElementById(id);
    if (!target || !settingsPage.contains(target)) return;
    search.value = ''; filter();
    const details = target.closest('details') || target.querySelector('details');
    if (details) details.open = true;
  };
  window.addEventListener('hashchange', revealHash);
  filter(); revealHash();
}

// Use the website's three-state footer control and shared storage preference.
const footerThemeButtons = document.querySelectorAll('[data-theme-value]');
function selectTheme(value) {
  const theme = value === 'light' || value === 'dark' ? value : 'auto';
  document.body.dataset.theme = theme;
  try {
    localStorage.setItem('theme', theme); // Keep Furo's initial theme script in sync.
    if (theme === 'auto') localStorage.removeItem('shot-theme');
    else localStorage.setItem('shot-theme', theme);
  } catch { /* The controls still work when storage is disabled. */ }
  footerThemeButtons.forEach(button => button.setAttribute('aria-pressed',
    String(button.dataset.themeValue === (theme === 'auto' ? 'system' : theme))));
}
let savedTheme = 'system';
try { savedTheme = localStorage.getItem('shot-theme') || 'system'; } catch { /* Default to system. */ }
selectTheme(savedTheme);
footerThemeButtons.forEach(button => button.addEventListener('click', () => selectTheme(button.dataset.themeValue)));


// Keep definition prose and signatures fluid without altering code blocks.
document.querySelectorAll('article dt.sig br').forEach(br => br.replaceWith(' '));

// Top-level API classes use native disclosure controls. Preserve all anchor IDs
// when moving the signature so existing member links continue to work.
if (/\/reference\/(python|cpp)\.html$/.test(location.pathname)) {
  const declarations = Array.from(document.querySelectorAll('article dl.py, article dl.cpp'));
  declarations.filter(dl => !dl.parentElement.closest('dl.py, dl.cpp')).forEach(dl => {
    const signatures = Array.from(dl.querySelectorAll(':scope > dt.sig'));
    const description = dl.querySelector(':scope > dd');
    if (!signatures.length || !description) return;
    const details = document.createElement('details'); details.className = 'api-entry';
    const summary = document.createElement('summary'); summary.className = 'sig';
    signatures.forEach(dt => {
      const signature = document.createElement('span'); signature.className = 'api-signature';
      if (dt.id) signature.id = dt.id;
      signature.append(...dt.childNodes); summary.append(signature);
    });
    const content = document.createElement('div'); content.className = 'api-entry-content';
    content.append(...description.childNodes);
    details.append(summary, content); dl.replaceWith(details);
  });
  function openApiTarget() {
    let target;
    try { target = document.getElementById(decodeURIComponent(location.hash.slice(1))); }
    catch { return; }
    if (!target) return;
    let parent = target.closest('details');
    while (parent) { parent.open = true; parent = parent.parentElement.closest('details'); }
    requestAnimationFrame(() => target.scrollIntoView({block: 'start'}));
  }
  window.addEventListener('hashchange', openApiTarget);
  openApiTarget();
}
