Managing openrave.org
=====================

openrave.org uses Django for managing documentation, news, and blogs. The code is maintained at::

   https://openrave.svn.sourceforge.net/svnroot/openrave/openrave.org

Using system similar to `djangoproject.com <https://github.com/django/djangoproject.com>`_

Install
=======

1. Create a virtualenv
2. Install dependencies:

.. code-block:: bash

  apt-get install postgresql postgresql-client libpq-dev memcached python-dev
  pip install -r deploy-requirements.txt

If you only need to deploy, and don't need to test any changes, you can use local-requirements.txt


3. Set up databases, as per django_website/settings/www.py

4. Create a 'openrave.org_secrets.json' file in the directoy above the checkout, containing
   something like::

    { "secret_key": "xyz",
      "superfeedr_creds": ["any@email.com", "some_string"] }

5. Initial DB Setup::

    ./manage.py syncdb
    ./manage.py convert_to_south docs

Future DB Update::

    ./manage.py syncdb
    ./manage.py migrate

6. For Docs::

  ./manage.py loaddata doc_releases.json
  ./manage.py update_docs 

7. For adding new document::

  DJANGO_SETTINGS_MODULE=openrave_website.settings python -c "from openrave_website.docs import models; models.DocumentRelease.objects.create(lang='en',version='0.7.0', scm=models.DocumentRelease.SVN, scm_url='https://openrave.svn.sourceforge.net/svnroot/openrave/tags/0.7.0', is_default=False);"

Re-index the documents::

  ./manage.py update_docs


8. Finally::

    python manage.py runserver

Creating PostgreSQL Database
============================

.. code-block:: bash

  sudo -u postgres psql --command "CREATE ROLE openrave PASSWORD 'testpass' SUPERUSER CREATEDB CREATEROLE INHERIT LOGIN;"
  createdb --host localhost --username openrave --encoding UTF-8 openrave_website

Editing
=======

All HTML templates are in **openrave_website/templates**. `Tutorial on HTML + Django syntax template language <https://docs.djangoproject.com/en/1.4/topics/templates/>`_

Only write English in the HTML files and only inside these translation blocks:

- `trans <https://docs.djangoproject.com/en/1.4/topics/i18n/translation/#std:templatetag-trans>`_

- `blocktrans <https://docs.djangoproject.com/en/1.4/topics/i18n/translation/#blocktrans-template-tag>`_  

Videos and image filenames should also be written within the translation blocks so that we can substitute them with the language equivalent.

Translating to Japanese
=======================

When English templates are done, execute::

  django-admin.py makemessages --locale=ja_JP

Open **locale/ja_JP/LC_MESSAGES/django.po** and edit the translations. When done execute::

  django-admin.py compilemessages --locale=ja_JP

Restart the mujinwww server and the new translation should be visible!

