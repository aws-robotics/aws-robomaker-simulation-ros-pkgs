import boto3
import botocore
from botocore.exceptions import NoCredentialsError

import datetime
from dateutil.tz import tzlocal


def RefreshableSession(credential_file=None, profile_name=None):
    '''
    Create a boto3 session that always refreshes (reloads) credentials via shared credential file.

    :type creds_filename: str
    :param creds_filename: A string to the shared credentials file. See
        botocore.credentials.SharedCredentialProvider

    :type profile_name: str
    :param profile_name: The name of the profile in the shared credential file. Default is 'default'.
    '''

    session = botocore.session.get_session()
    credential_file = credential_file if credential_file else session.get_config_variable('credentials_file')

    # Insert custom provider first, before all other providers during provider chain resolution
    # Inspired by https://github.com/boto/boto3/issues/443
    custom_provider = CustomRefreshedSharedCredentialProvider(creds_filename=credential_file, profile_name=profile_name)
    providers = session.get_component('credential_provider')
    providers.insert_before('env', custom_provider)

    return boto3.session.Session(botocore_session=session)


class AlwaysReloadCredentials(botocore.credentials.DeferredRefreshableCredentials):
    '''
    Always reload credentials from the shared credential file

    We need this as containers do not know the expiry time for the 
    shared credentials written by the agent. If we use an arbitrary
    expiry time, there will be periods between new credentials written
    by the agent and refreshing. Instead, always read the file. Yes,
    this is slow but today, these APIs are not used rapidly.
    '''
    def __init__(self, refresh_using=None, method=None):
        super(AlwaysReloadCredentials, self).__init__(refresh_using, method)

    def refresh_needed(self, refresh_in=None):
        return True

    def _is_expired(self):
        # Without this, we need an expiry time and incurs a (slight) race
        # condition between load and is_expired().
        # See https://github.com/boto/botocore/blob/8bff27c9550f707d9b90734af5442438296db943/botocore/credentials.py#L516-L534,
        # where 516 invokes load(), then line 534 `is_expired()` is never true.
        return False


class CustomRefreshedSharedCredentialProvider(botocore.credentials.CredentialProvider):
    METHOD = botocore.credentials.SharedCredentialProvider.METHOD
    CANONICAL_NAME = "CustomRefreshedSharedCredentials"

    def __init__(self, creds_filename, profile_name=None):
        """
        Create a shared credentials provider always loads from the shared credentials
        file.

        :type creds_filename: str
        :param creds_filename: A string to the shared credentials file. See
            botocore.credentials.SharedCredentialProvider

        :type profile_name: str
        :param profile_name: The name of the profile in the shared credential file. Default is 'default'.
        """
        self._provider = botocore.credentials.SharedCredentialProvider(creds_filename, profile_name)

    def load(self):
        return AlwaysReloadCredentials(refresh_using=self._refresh, method=self.METHOD)

    def _refresh(self):
        creds = self._provider.load()
        if creds is None:
            raise NoCredentialsError()
        return {
            "access_key": creds.access_key,
            "secret_key": creds.secret_key,
            "token": creds.token,
            "expiry_time": (datetime.datetime.now(tzlocal()) + datetime.timedelta(seconds=1)).isoformat() # in the future
        }
